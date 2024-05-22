classdef vehicle < handle
    properties

        %Parametres Control
        x = 0;
        y = 0;
        theta = 0;
        gamma = 0; %angle of the front wheel
        v = 0; %linear speed

        id_vehicle = 0; 
        lbase = 3; %distance between front and back wheels
        
        dt = 0.1; % default discretisation time
        
        %handler
        targets = []; %target objects
        obstacles = []; %obstacle objects
        vehicles = []; %vehicle objects

        %controllers memory
        actual_target;
        obstacle_to_avoid;
        %vehicle_to_follow;
        limitcycles=0;

        %arrays for correction action visualisation
        theta_error_output=[];
        speed_output=[];
        lyap1=[];
        lyap2=[];
        lyap3=[];

        %plot parameters (only for debug)
        L = 2.5;     % Empattement
        l = 3;  % Largeur des essieux
        r = 1;     % Rayon des roues

        %Parametres Ordonancement
        id_road;
        dist_from_start;
        nbpassengers;
        plannedDemands;
        actualPath;
        priority;


    end
    
    methods
        %constructor
        function obj = vehicle(x, y, theta, gamma, obstacles, targets, vehicles, id_vehicle, id_road, dist_from_start)
             
            if nargin == 10
                %Control
                obj.x = x;
                obj.y = y;
                obj.theta = theta;
                obj.gamma = gamma;
                obj.obstacles = obstacles;
                obj.vehicles = vehicles;

                %% filtrer targets and remove targets that are above obstacles
                % for loop on every targets / obstacles
                obj.targets = targets;
                
                %Ordonancement
                obj.id_vehicle = id_vehicle;
                obj.id_road = id_road;
                obj.dist_from_start = dist_from_start;
                obj.nbpassengers = 0;
                obj.plannedDemands = [];
                obj.actualPath = NaN;
                obj.priority = NaN;
            else
                 error('Incorrect number of arguments for vehicle constructor');
            end
         end
        
        %plot the vehicle pos - debug mode
        function plot(obj)
                xArD = obj.x + obj.l/2*sin(obj.theta); % right back wheel
                yArD = obj.y - obj.l/2*cos(obj.theta);
                xArDDev = xArD + obj.r*cos(obj.theta);
                yArDDev = yArD + obj.r*sin(obj.theta);
                xArDArr = xArD - obj.r*cos(obj.theta);
                yArDArr = yArD - obj.r*sin(obj.theta);
                
                xArG = obj.x - obj.l/2*sin(obj.theta); % left back wheel
                yArG = obj.y + obj.l/2*cos(obj.theta);
                xArGDev = xArG + obj.r*cos(obj.theta);
                yArGDev = yArG + obj.r*sin(obj.theta);
                xArGArr = xArG - obj.r*cos(obj.theta);
                yArGArr = yArG - obj.r*sin(obj.theta);

                if obj.id_vehicle==1
                    color = "r";
                else
                    color = "b";
                end

                %vehicle lines
                line([xArG xArD],[yArG yArD])
                line([xArGDev xArGArr],[yArGDev yArGArr],'Color',color,'LineWidth',3)
                line([xArDDev xArDArr],[yArDDev yArDArr],'Color',color,'LineWidth',3)
                
                for i=0:0.2:(2*pi)
                    plot(obj.x+(obj.l/2)*cos(i),obj.y+(obj.l/2)*sin(i),'-','LineWidth',3);
                end
        end

        function plot2(obj)
            if obj.id_vehicle==1
                    color = "r+";
                else
                    color = "b+";
            end
            plot(obj.x, obj.y, color, 'LineWidth', 2);
        end
        
        function update(obj, dt, sched)
            %{
            cette fonction calcule la cible à atteindre
            prépare les datas pour la loi de commande
            recupère la commande
            et actualise la position
            %}
            obj.dt = dt;

            if isempty(obj.targets)
                return;
            end

            obj.target_selection() %update obj.actual_target
            control = obj.control_law(); % return controller command
            obj.set_pos(control) % update vehicle law
            if ~isnan(sched)
                obj.updatePosition(sched);
            end
        end
        
        function target_selection(obj)
            %{
            cette fonction regarde si il a des obstacles à eviter
            sinon si il y a un vehicule a suivre
            sinon on suit les cibles prédéfinis par l'utilisateurs
            
            on défini alors la cible défini par une des 3 situations
            et on la suit
            %}

            %reset obstacle to avoid and vehicle to follow
            obj.obstacle_to_avoid = obstacle(0,0,0,0);
            %obj.vehicle_to_follow = vehicle(0,0,0,0,[],[],[],0);

            % ------ Remove reached targets ---------
            tar = [obj.targets(1).theta, obj.targets(1).x, obj.targets(1).y];
            T_O_T = [cos(tar(1)) -sin(tar(1)) tar(2)
                     sin(tar(1)) cos(tar(1))  tar(3)
                     0           0            1]; %transformation matrix

            vehicle_base_vehicule = T_O_T\[obj.x; obj.y; 1];
            
            x_vehicle_base_target = vehicle_base_vehicule(1);
            error_distance = obj.get_distance_object(obj.targets(1));
            error_theta = abs(obj.theta-obj.targets(1).theta);
            
            % check if the target is already reached
            if ( ((error_theta<=1.5) && (error_distance<=1)) || (x_vehicle_base_target>0))
                obj.targets(1)=[];
            end
            
            if ~isempty(obj.targets)
                obj.actual_target = obj.targets(min(3, length(obj.targets)));
            else 
                return;
            end
            
            

            % ------ Check if there is an obstacle to avoid ---------
            minimal_distance=-1;
            for i=1:size(obj.obstacles,2)
                dist = obj.get_distance_object(obj.obstacles(i));
                if ( dist<=(obj.obstacles(i).getRayonInfluence()) && dist<= minimal_distance )
                    minimal_distance = dist;
                    obj.obstacle_to_avoid = obj.obstacles(i);

                    %obj.actual_target = obj.compute_limited_cycles
                end
            end
            


            % ------ Check if there is a vehicle to follow ---------
            for i=1:size(obj.vehicles,2)
                dist=obj.get_distance_object(obj.vehicles(i));
                
                veh = [obj.theta, obj.x, obj.y];
                V_O_V = [cos(veh(1)) -sin(veh(1)) veh(2)
                         sin(veh(1)) cos(veh(1))  veh(3)
                         0           0            1]; %transformation matrix

                vehicle_base_vehicule = V_O_V\[obj.vehicles(i).x; obj.vehicles(i).y; 1];
                
                x_vehicle_base_vehicle = vehicle_base_vehicule(1);
                
                if ~isempty(obj.vehicles(i).targets)
                    diff_target = abs(obj.targets(1).theta-obj.vehicles(i).targets(1).theta);
                else
                    diff_target = 1;
                end

                if (dist <= 30 && x_vehicle_base_vehicle>0 && diff_target<0.1)
                    obj.actual_target = obj.compute_vehicle_offset(obj.vehicles(i));
                end
            end
        end
        
        function control=control_law(obj)
            vmax=50/3.6; % 50km/h

            curvature_t=obj.actual_target.getCurv;

            %compute the datas for the attraction controller
            xe = obj.actual_target.x - obj.x;
            ye = obj.actual_target.y - obj.y;
            error_x = cos(obj.theta)*xe + sin(obj.theta)*ye;
            error_y = -sin(obj.theta)*xe + cos(obj.theta)*ye;
            
            error_theta = subtr_ang(obj.actual_target.theta, obj.theta);
            
            thetaRT = atan2(ye,xe);
            error_RT = subtr_ang(obj.actual_target.theta, thetaRT);

            d=sqrt(error_x^2 + error_y^2);

            if error_y > 0.1
                curvature_t = 0;
            end

            %Controller parameters
            K_x = 0.15;      %Main for the overall velocity
            K_d = 10;      %Main for Ex (d in the Lypauniv function)
            K_l = 2;      %Main for "d", error E_RT and E_theta
            K_o = 2;      %Main for E_theta and C_c ATTENTION NE DOIT PAS ETRE EGAL A "0", On divise sur K_o %%0.3;%1;%1.2/distf steering
            K_theta = 5;  %Main for E_Theta 0.8;%1.0
            K_rt = 0.001;     %Main for E_RT %%0.01
            
            %more oscillation but faster
            % K_x = 0.3;    %Main for the overall velocity
            % K_d = 10;     %Main for Ex (d in the Lypauniv function)
            % K_l = 5;      %Main for "d", error E_RT and E_theta
            % K_o = 2;      %Main for E_theta and C_c ATTENTION NE DOIT PAS ETRE EGAL A "0", On divise sur K_o %%0.3;%1;%1.2/distf steering
            % K_theta = 5;  %Main for E_Theta 0.8;%1.0
            % K_rt = 0.01;  %Main for E_RT %%0.01


            CosE_theta = cos(error_theta);
            SinE_theta = sin(error_theta);

            SinE_RT = sin(error_RT);
            CosE_RT = cos(error_RT);

            curv =   curvature_t/CosE_theta + ... %%1st term
                     (d^2*curvature_t*K_l*SinE_RT*CosE_RT)/(K_o*SinE_theta*CosE_theta)+ ...%%2sd term
                     K_theta*(SinE_theta/CosE_theta)+ ... %%3rd term
                     (K_d*error_y - K_l*d*SinE_RT*CosE_theta)/(K_o*CosE_theta)+... %%4rd term
                     (K_rt*SinE_RT^2)/(SinE_theta*CosE_theta);

            if isnan(curv) %% CASE WHERE THERE IS A PROBLEME OF COMPUTATIONS
                curv = 0.0001;
            end

            vb=  K_x*(K_d*error_x + K_l*d*SinE_RT*sin(error_theta) + K_o*sin(error_theta)*curv);
            V =  obj.actual_target.v*cos(error_theta) + vb;

            if ( (isnan(vb)) || (abs(V)> vmax)) %saturation of linear velocity
                V = sign(V)*vmax/2;
            end

            gamma_controller = atan(obj.lbase*curv);

            %Calcul des 3 termes li�s � la fonction de Lyapunov
            VFLyap1 = 0.5*d^2*K_d;
            VFLyap2 = 0.5*d^2*K_l*SinE_RT^2;
            VFLyap3 = K_o*(1- CosE_theta);

            % store controller effects for plot
            obj.theta_error_output = [obj.theta_error_output;[obj.theta obj.actual_target.theta]];
            obj.speed_output = [obj.speed_output; V];
            obj.lyap1 = [obj.lyap1; VFLyap1];
            obj.lyap2 = [obj.lyap2; VFLyap2];
            obj.lyap3 = [obj.lyap3; VFLyap3];
            
            control = [V , gamma_controller];
        end


        function offset=compute_vehicle_offset(obj, vehicle)
            disp([obj.id_vehicle,"following", vehicle.id_vehicle])
            error_y = vehicle.y-obj.y;
            error_x = vehicle.x-obj.x;
            phi = atan2(error_y, error_x);

            r = 4
            x_offset = vehicle.x-cos(phi)*4;
            y_offset = vehicle.y-sin(phi)*4;

            offset = target(x_offset, y_offset, vehicle.theta, 0, 0);
        end
        
        
        
        function avoiding_target=compute_limited_cycles(obj, obstacle)
            %% FOR RUBEN : 

            %% Edit this function

            %compute the datas for the obstacle avoidance controller
            error_x = obj.x - obj.obstacle_to_avoid.x;
            error_y = obj.y - obj.obstacle_to_avoid.y;
            
            %transformation matrix
            X_D_O = obj.actual_target.x - obj.obstacle_to_avoid.x;
            Y_D_O = obj.actual_target.y - obj.obstacle_to_avoid.y;
            alpha = atan2(Y_D_O, X_D_O);
            T_O_A = [cos(alpha) -sin(alpha) 0 obj.obstacle_to_avoid.x
                     sin(alpha)  cos(alpha) 0 obj.obstacle_to_avoid.y
                     0           0          1 0
                     0           0          0 1];
            
            obstacle_coords = T_O_A\[obj.x; obj.y; 0; 1];
            X_obst = obstacle_coords(1);
            
            % get rayoncyclelimite
            if (X_obst <= 0)
                obj.limitcycles=(obj.obstacle_to_avoid.getRayonInfluence+obj.l)-0.3;
            else
                obj.limitcycles=obj.limitcycles+0.03;
            end
            
            x_dot = error_y + error_x * ((obj.limitcycles^2) - (error_x^2) - (error_y^2));
            y_dot = -error_x + error_y * ((obj.limitcycles^2) - (error_x^2) - (error_y^2));
            theta_dot = atan2(y_dot, x_dot);
            
            X0 = [x_dot, y_dot];
            xc = ode23(@(t, y) EquationDiff_Tourbillon(t, y, obj.limitcycles), [0, 0.2], X0);
            theta_controller = atan2((xc.y(2, 2) - xc.y(1, 2)), (xc.y(2, 1) - xc.y(1, 1)));
            error_theta=SoustractionAnglesAtan2(theta_dot, obj.theta);
            
            avoiding_target = [error_theta; theta_controller];
        end
        
        
        function set_pos(obj, control)
            %update the position of the vehicle
            obj.v = control(1);
            tmp_gamma = control(2) / 5;

            obj.x = obj.x + obj.v * cos(obj.theta) * obj.dt;
            obj.y = obj.y + obj.v * sin(obj.theta) * obj.dt;
            obj.theta = obj.theta + (tan(tmp_gamma/obj.lbase)) * obj.v * obj.dt;
            obj.gamma = tmp_gamma;
        end
        
        function dist=get_distance_object(obj, object)
            dist=sqrt((obj.x-object.x)^2+(obj.y-object.y)^2);
        end

        function update_acc_vehicles(obj, vehicles)
            obj.vehicles = vehicles;
        end

        function plot_corrector_action(obj)
            figure(obj.id_vehicle*5+1);
            plot(obj.theta_error_output(:,1),'red')
            hold on;
            plot(obj.theta_error_output(:,2),'blue')
            legend('sortie','consigne')
            title('Correction angle du véhicule')
            
            figure(obj.id_vehicle*5+2);
            plot(obj.speed_output(:,1),'red')
            legend('speed output')
            title('Evolution de la vitesse')

            figure(obj.id_vehicle*5+3);
            plot(obj.lyap1(:,1),'red')
            legend('lyapunov')
            title('Evolution Lyap1')

            figure(obj.id_vehicle*5+4);
            plot(obj.lyap2(:,1),'red')
            legend('lyapunov')
            title('Evolution Lyap2')

            figure(obj.id_vehicle*5+5);
            plot(obj.lyap3(:,1),'red')
            legend('lyapunov')
            title('Evolution Lyap3')
        end



        %% Ordonancement ----------
        function obj = addDemand(obj,demand)
            obj.plannedDemands = [obj.plannedDemands;demand];
        end


        function obj = addPassenger(obj,nb)
            obj.nbpassengers = obj.nbpassengers + nb;
        end

        function obj = updatePosition(obj,schedul)           
            % gestion des demandes en cours
            for i = 1:size(obj.plannedDemands,1)
                % vérification du fait que l'on soit au point de départ
                % d'une demande
                pos = get_pos_by_id(schedul.modified_nav,obj.plannedDemands(i).id_dep);
                if obj.plannedDemands(i).visited_dep == false && ...
                    (schedul.modified_nav.States.StateVector(pos,1)-2<obj.y && obj.y<schedul.modified_nav.States.StateVector(pos,1)+2) && ...
                    (schedul.modified_nav.States.StateVector(pos,2)-2<obj.x && obj.x<schedul.modified_nav.States.StateVector(pos,2)+2)
                    disp(obj.targets);
                    disp("Visited");
                    disp(obj.targets);
                    obj.plannedDemands(i).visited_dep = true;
                end

                % vérification si on est au point d'arrivée d'une demande
                % et supression de la demande si on y est
                

                if (schedul.modified_nav.States.StateVector(pos,1)-2<obj.x && obj.x<schedul.modified_nav.States.StateVector(pos,1)+2) && ...
                    (schedul.modified_nav.States.StateVector(pos,2)-2<obj.y && obj.y<schedul.modified_nav.States.StateVector(pos,2)+2) && ...
                    (obj.plannedDemands(i).visited_dep == true)
                    obj.addPassenger(obj.plannedDemands(i).nbpassengers *(-1));
                    obj.plannedDemands = [obj.plannedDemands(1:i-1);obj.plannedDemands(i+1:end)];
                    obj.updatePriority();                   
                end
                    
            end
            
        end

        function obj = updatePriority(obj)
            obj.priority = -1;
            for i = 1:size(obj.plannedDemands,1)
                if obj.plannedDemands(i).priority > obj.priority
                    obj.priority = obj.plannedDemands(i).priority;
                end
            end
            if obj.priority == -1
                obj.priority = NaN;
            end
        end
        
        function [id] = getIdVehicle(obj)
            id = obj.id_vehicle;
        end
        function [id_road]=getRoad(obj)
            id_road = obj.id_road;
        end
        function [X]=getX(obj)
            X = obj.x;
        end
        function [Y]=getY(obj)
            Y = obj.y;
        end
        function [dist]=getDistFromStart(obj)
            dist = obj.dist_from_start;
        end
        function obj = assignateNewPath(obj,path)
            obj.actualPath = path;
        end 
        function [dist] = getTotalCost(obj,demands,NavG)
            % calcul des coûts de tous les trajets possible entre la
            % position d'un véhicule et toutes les demandes
            if isempty(demands)
                error("Demands tab is empty");
            end

            pos = find(NavG.Links.Id_route == obj.id_road);

            id_dep = NavG.Links.EndStates(pos,2);
            combs = getCombs(id_dep,demands);
            dual_combs = getdualcombs(combs);
            tmp1 = [];
            tmp2 = [];
            for i = 1:size(dual_combs,1)
                [pathOutput,solutionInfo] = GetPath(NavG,get_pos_by_id(NavG,dual_combs(i,1)),get_pos_by_id(NavG,dual_combs(i,2))); 
                tmp1 = [tmp1; Path(pathOutput, solutionInfo.PathCost, NavG, obj.id_vehicle)];
                tmp2 = [tmp2;solutionInfo.PathCost];
            end
            dual_dist = table(dual_combs, tmp1, tmp2,VariableNames=["dual_combs","Path","Path_Cost"]);
            total_cost=[];
            total_path = [];
            for i = 1:size(combs,1) 
                comb = combs(i,:);
                c = [comb(1:length(comb)-1); comb(2:length(comb))]';
                cost = [];
                path = [];
                for j = 1:size(c,1)
                    pair = find(ismember(dual_dist{:, {'dual_combs'}}, [c(j,1), c(j,2)] , 'rows'));
                    cost = [cost;dual_dist(pair,3)];
                    path = [path;dual_dist(pair,2)];
                end
                total_cost = [total_cost;sum(cost)]; 
                total_path = [total_path;assoicatePaths(path,table2array(total_cost),NavG,obj.id_vehicle)];
            end
            dist = table(combs,total_path,total_cost,VariableNames=["Combinaisons","Chemin","Cout_total"]);
        end

        function [res] = getOptimalPath(obj,demands,NavG)
            % Retourne une table contenant le chemin optimal dans lequel a été ajouté le
            % nouveau trajet sous la forme suivante (combinaison des points de passages,
            % Chemin sous forme de Path, cout total)
            if isempty(demands)
                error("Demands tab isf empty");
            end
            
            dist = obj.getTotalCost(demands,NavG);
            [~, id_min] = min(dist.Cout_total); 
            min_row = dist(table2array(id_min), :);
            comb = min_row{1, 'Combinaisons'}; 
            optimal_path = min_row{1, 'Chemin'}; 
            cost_total = table2array(min_row{1, 'Cout_total'}); 
            res = table(comb, optimal_path, cost_total, 'VariableNames', {'Combinaisons', 'Chemin', 'Cout_total'});
        end

        function showCarPath(obj,NavG)
            h = show(NavG);
            set(h,XData=NavG.States.StateVector(:,1), ...
                YData=NavG.States.StateVector(:,2));
            pathStateIDs = [];
            if ~isnan(obj.actualPath)
                for i = 1:size(obj.actualPath.points,1)
                    pathStateIDs = [pathStateIDs;get_pos_by_id(NavG,recherche_dans_nodes(obj.actualPath.points(i,1),obj.actualPath.points(i,2),NavG))];
                end
            else
                pos = find(NavG.Links.Id_route == obj.id_road);
                id_dep = NavG.Links.EndStates(pos,2);
                pathStateIDs = [get_pos_by_id(NavG,id_dep)];
            end
            highlight(h,pathStateIDs,EdgeColor="#EDB120",LineWidth=4);
            highlight(h,pathStateIDs(1),NodeColor="#77AC30",MarkerSize=5);
            highlight(h,pathStateIDs(end),NodeColor="#D95319",MarkerSize=5);
        end
    end
end