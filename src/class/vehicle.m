classdef vehicle < handle
    properties
        x = 0;
        y = 0;
        theta = 0;
        gamma = 0; %angle of the front wheel
        v = 0; %linear speed

        id = 0; 
        lbase = 3; %distance between front and back wheels
        
        dt = 0.1; % default discretisation time
        
        %handler
        targets = []; %target objects
        obstacles = []; %obstacle objects
        vehicles = []; %vehicle objects

        %controllers memory
        actual_target;
        obstacle_to_avoid;
        vehicle_to_follow;
        limitcycles=0;

        %arrays for correction action visualisation
        theta_error_output=[];
        speed_output=[];

        %plot parameters (only for debug)
        L = 2.5;     % Empattement
        l = 3;  % Largeur des essieux
        r = 1;     % Rayon des roues
    end
    
    methods
        %constructor
        function obj = vehicle(x, y, theta, gamma, obstacles, targets, vehicles, id)
            if nargin == 8
                obj.x = x;
                obj.y = y;
                obj.theta = theta;
                obj.gamma = gamma;
                obj.obstacles = obstacles;
                obj.vehicles = vehicles;
                obj.id = id;

                %% filtrer targets and remove targets that are above obstacles
                % for loop on every targets / obstacles

                obj.targets = targets;
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

                %orientation :
                plot([obj.x, obj.x + cos(obj.theta) * obj.lbase ], [obj.y, obj.y + sin(obj.theta) * obj.lbase],'Color','r','LineWidth',3);
                
                %vehicle lines
                plot(obj.x,obj.y,'r+');
                line([xArG xArD],[yArG yArD])
                line([xArGDev xArGArr],[yArGDev yArGArr],'Color','r','LineWidth',3)
                line([xArDDev xArDArr],[yArDDev yArDArr],'Color','r','LineWidth',3)
                
                for i=0:0.2:(2*pi)
                    plot(obj.x+(obj.l/2)*cos(i),obj.y+(obj.l/2)*sin(i),'-','LineWidth',3);
                end
        end
        
        function update(obj, dt)
            %{
            cette fonction calcule la cible à atteindre
            prépare les datas pour la loi de commande
            recupère la commande
            et actualise la position
            %}
            obj.dt = dt;

            obj.target_selection() %update obj.actual_target
            control = obj.control_law(); % return controller command
            obj.set_pos(control) % update vehicle law
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
            obj.vehicle_to_follow = vehicle(0,0,0,0,[],[],[],0);

            % ------ Remove reached targets ---------
            tar = [obj.targets(1).theta, obj.targets(1).x, obj.targets(1).y];
            T_O_T = [cos(tar(1)) -sin(tar(1)) tar(2)
                     sin(tar(1)) cos(tar(1))  tar(3)
                     0           0            1]; %transformation matrix

            vehicle_base_target = T_O_T\[obj.x; obj.y; 1];
            
            x_vehicle_base_target = vehicle_base_target(1);
            error_distance = obj.get_distance_object(obj.targets(1));
            error_theta = abs(obj.theta-obj.targets(1).theta);
            
            % check if the target is already reached
            if ( ((error_theta<=1.5) && (error_distance<=1)) || (x_vehicle_base_target>100))
                obj.targets(1)=[];
            end
            
            if ~isempty(obj.targets)
                obj.actual_target = obj.targets(1);
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
                diff_ang=abs(obj.theta-obj.vehicles(i).theta);
                if (dist <= 30 && diff_ang <= 0.7)
                    obj.actual_target = obj.compute_vehicle_offset(obj.vehicles(i));
                end
            end
        end
        
        function control=control_law(obj)
            disp("-----")
            disp(obj.id)
            disp(obj.actual_target)
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
            K_x = 0.3;    %Main for the overall velocity
            K_d = 10;     %Main for Ex (d in the Lypauniv function)
            K_l = 5;      %Main for "d", error E_RT and E_theta
            K_o = 2;      %Main for E_theta and C_c ATTENTION NE DOIT PAS ETRE EGAL A "0", On divise sur K_o %%0.3;%1;%1.2/distf steering
            K_theta = 5;  %Main for E_Theta 0.8;%1.0
            K_rt = 0.01;  %Main for E_RT %%0.01

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

            % store controller effects for plot
            obj.theta_error_output = [obj.theta_error_output;[obj.theta obj.actual_target.theta]];
            obj.speed_output = [obj.speed_output; V];
            
            control = [V , gamma_controller];
        end


        function offset=compute_vehicle_offset(obj, vehicle)
            error_y = vehicle.y-obj.y;
            error_x = vehicle.x-obj.x;
            phi = atan2(error_y, error_x);

            x_offset = vehicle.x-cos(phi)*5;
            y_offset = vehicle.y-sin(phi)*5;

            offset = target(x_offset, y_offset, vehicle.theta, vehicle.v, 0);
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
            tmp_gamma = control(2);

            obj.x = obj.x + obj.v * cos(obj.theta) * obj.dt;
            obj.y = obj.y + obj.v * sin(obj.theta) * obj.dt;
            obj.theta = obj.theta + (tan(tmp_gamma/obj.lbase)) * obj.v * obj.dt;
            obj.gamma = tmp_gamma;
        end
        
        function dist=get_distance_object(obj, object)
            dist=sqrt((obj.x-object.x)^2+(obj.y-object.y)^2);
        end
    end
end