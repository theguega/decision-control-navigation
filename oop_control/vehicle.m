classdef vehicle < handle
    properties
        x = 0;
        y = 0;
        theta = 0;
        v = 0; %linear speed
        w = 0; %angular speed
        
        %vehicle parameters
        L = 2.5;     % Empattement
        d = 3;  % Largeur des essieux
        r = 1;     % Rayon des roues
        
        dt=0.1;
        
        to_avoid = obstacle(0,0,0,0); %obstacle to avoid
        distance_to_avoid=0;
        
        RayonCycleLimite=0;
        
        controller = 1;
    end
    
    methods
        %constructor
        function obj = vehicle(x, y, theta, v, w)
            obj.x = x;
            obj.y = y;
            obj.theta = theta;
            obj.v = v;
            obj.w = w;
        end
        
        %plot the actual position of the vehicle
        function plot(obj)
            %===================
            % Essieu Arriere  ||
            %===================
            % --- Centre de la roue arriere droite
            xArD = obj.x + obj.d/2*sin(obj.theta);
            yArD = obj.y - obj.d/2*cos(obj.theta);
            %
            % % --- Centre de la roue arriere gauche
            xArG = obj.x - obj.d/2*sin(obj.theta);
            yArG = obj.y + obj.d/2*cos(obj.theta);
            
            %========================
            % Roue Arriere Gauche  ||
            %========================
            xArGDev = xArG + obj.r*cos(obj.theta);
            yArGDev = yArG + obj.r*sin(obj.theta);
            
            xArGArr = xArG - obj.r*cos(obj.theta);
            yArGArr = yArG - obj.r*sin(obj.theta);
            %
            % %========================
            % % Roue Arriere Droite  ||
            % %========================
            xArDDev = xArD + obj.r*cos(obj.theta);
            yArDDev = yArD + obj.r*sin(obj.theta);
            
            xArDArr = xArD - obj.r*cos(obj.theta);
            yArDArr = yArD - obj.r*sin(obj.theta);
            
            %===========================
            % Trace du Robot Mobile  ||
            %===========================
            figure(1)
            hold on
            plot(obj.x,obj.y,'r+');              % Trace du centre de l'essieu arriere
            grid on
            %line([xe xA],[ye yA])         % Trace de l'empattement -- relie (x,y) e (xA,yA)
            %plot(xA, yA, 'b+')            % Trace du devant du robot
            line([xArG xArD],[yArG yArD])                           % Trace de l'essieu arriere -- relie (xArG,yArG) e (xArD,yArD)
            line([xArGDev xArGArr],[yArGDev yArGArr],'Color','r','LineWidth',2)   % Trace de la roue arriere gauche -- relie (xArGDev,yArGDev) e (xArGArr,yArGArr)
            line([xArDDev xArDArr],[yArDDev yArDArr],'Color','r','LineWidth',2)   % Trace de la roue arriere droite -- relie (xArDDev,yArDDev) e (xArDArr,yArDArr)
            
            for i=0:0.2:(2*pi)
                plot(obj.x+(obj.d/2)*cos(i),obj.y+(obj.d/2)*sin(i),'-','LineWidth',2);
            end
        end
        
        
        
        %choose the right controller depending on the situation
        function CommandeReelle=controller_selection(obj, obstacles, target)
            %reset obstacle to avoid
            obj.to_avoid = obstacle(0,0,0,0);
            obj.distance_to_avoid=-1;
            
            %check if there is an obstacle to avoid
            for i=1:size(obstacles,2)
                dist = sqrt( (obstacles(i).getX() - obj.x)^2 + (obstacles(i).getY() - obj.y)^2);
                if dist<=obstacles(i).getRayonInfluence()
                    %if the actual obstacle is neareast than the previous one
                    if dist<obj.distance_to_avoid
                        obj.to_avoid=obstacle(i);
                        obj.distance_to_avoid=dist;
                        obj.controller=2;
                    end
                end
            end
            
            % if there is an obstacle
            if obj.controller==2
                datas=obj.var_obstacle_avoidance(target);
            else
                datas=obj.var_attraction(target);
            end
            
            if obj.controller == 1
                disp("control_attraction")
                CommandeReelle=obj.control_attraction(datas);
            else
                disp("control_obstacle_avoidance")
                CommandeReelle=obj.control_obstacle_avoidance(datas);
            end
        end
        
        function datas=var_attraction(obj, target)
            %compute the datas for the attraction controller
            Ex = target(1) - obj.x;
            Ey = target(2) - obj.y;
            Ecart = sqrt((Ex^2)+(Ey^2));
            
            ThetaC=atan2(Ey,Ex);
            
            ThetaTilde=SoustractionAnglesAtan2(ThetaC, obj.theta);
            
            datas = [Ecart; ThetaTilde; Ex; Ey; obj.theta];
        end
        
        function datas=var_obstacle_avoidance(obj, target)
            %compute the datas for the obstacle avoidance controller
            Ex = obj.x - obj.to_avoid.getX();
            Ey = obj.y - obj.to_avoid.getY();
            
            %Methode pour le changenement de repere
            X_D_O = target(1) - obj.to_avoid.getX();
            Y_D_O = target(2) - obj.to_avoid.getY();
            Alpha = atan2(Y_D_O, X_D_O);
            
            %Calcul de la matrice de passage du repere obtstacle (R_O) au repere absolu (R_A)
            T_O_A = inv ([cos(Alpha) -sin(Alpha) 0 obj.to_avoid.getX()
                sin(Alpha) cos(Alpha) 0 obj.to_avoid.getY()
                0 0 1 0
                0 0 0 1]);
            
            CoordonneeRepereObstacle = T_O_A * [obj.x; obj.y; 0; 1];
            
            X_Prime = CoordonneeRepereObstacle(1); %Attention le "prime" ici ne veut pas dire derivee
            Y_Prime = CoordonneeRepereObstacle(2);
            
            % Obtention du Rayon du cycle-limite a suivre
            if (X_Prime <= 0) %Alors Attraction vers le cycle limite
                obj.RayonCycleLimite=obj.to_avoid.getRayonInfluence()-0.3;
            else %Periode de l'extraction du cycle limite
                obj.RayonCycleLimite=obj.RayonCycleLimite+0.03; %Pour que le changement de rayon soit en douceur.
            end
            
            X_dot = sign(Y_Prime) * obj.y + obj.x * ((obj.RayonCycleLimite^2) - (obj.x^2) - (obj.y^2));
            Y_dot = -obj.x + obj.y * ((obj.RayonCycleLimite^2) - (obj.x^2) - (obj.y^2));
            ThetaC = atan2(Y_dot, X_dot);
            
            X0 = [X_dot, Y_dot];
            Xc = ode23(@(t, y) EquationDiff_Tourbillon(t, y, obj.RayonCycleLimite), [0, 0.2], X0);
            ThetaC_p = atan2((Xc.y(2, 2) - Xc.y(1, 2)), (Xc.y(2, 1) - Xc.y(1, 1)));
            
            ThetaTilde=SoustractionAnglesAtan2(ThetaC, obj.theta);
            
            %Param�tres de commande
            datas = [ThetaTilde; ThetaC_p; sign(Y_Prime)];
        end
        
        function CommandeReelle=control_attraction(obj, datas)
            %Donnees = [Ecart; ThetaTilde; Ex; Ey; ThetaReel];
            Ecart = datas(1);
            %ThetaTilde = datas(2);
            Ex = datas(3);
            Ey = datas(4);
            ThetaReel = datas(5);
            
            %Vmax = 1; %1m/s -> 3.6km/h
            Vmax = 0.5;
            
            if (gt(Ecart,0.2)) % Pour appliquer cette commande uniquement quand le robot n'est pas tout pr�s de la cible
                %%La position du point effectif (dispos� sur le robot) � asservir sur la consigne
                l1 = 0.4; %Selon l'axe x (Devant le robot)
                %l2 = 0;   %Selon l'axe y (A c�t� du robot)
                K1 = 0.1;
                K2 = 0.1;
                %V1 =  K1*Ex;
                %V2 =  K2*Ey;
                %%
                M = [cos(ThetaReel), -l1*sin(ThetaReel);
                    sin(ThetaReel), l1*cos(ThetaReel)];
                E = [Ex;Ey];
                K = [K1;K2];
                Commande = K.*(inv(M)*E);
                %%
                V = Vmax;
                W = Commande(2);
            else
                V = 0;
                W = 0;
            end
            
            lyapunov = 0.5*(Ex^2+Ey^2);
            CommandeReelle = [V, W, lyapunov];
        end
        
        function CommandeReelle=control_obstacle_avoidance(obj, datas)
            %Donnees = [ThetaTilde; ThetaC_p];
            ThetaTilde = datas(1);
            ThetaC_p = datas(2);
            
            %Vmax = 1; %1m/s -> 3.6km/h
            Vmax = 0.5;
            
            Vmax = 0.5;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Premier type de commande
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            Kp = 15;
            V = Vmax;
            W = ThetaC_p + Kp*ThetaTilde;
            
            lyapunov = 0.5*rad2deg(ThetaTilde)^2/10;
            CommandeReelle = [V, W, lyapunov];
        end
        
        %setter
        function update_pos(obj, CommandeReelle)
            %update the position of the vehicle
            obj.v = CommandeReelle(1);
            obj.w = CommandeReelle(2);
            
            obj.x = obj.x + obj.v * cos(obj.theta) * obj.dt;
            obj.y = obj.y + obj.v * sin(obj.theta) * obj.dt;
            obj.theta = obj.theta + obj.w * obj.dt;
        end
        
        function dist=get_distance_object(obj, object)
            dist=sqrt((obj.x-object.getX())^2+(obj.y-object.getY())^2);
        end
        
        function dist=get_distance_point(obj, target)
            dist=sqrt((obj.x-target(1))^2+(obj.y-target(2))^2);
        end
        
        %getters
        function x = getX(obj)
            x = obj.x;
        end
        
        function y = getY(obj)
            y = obj.y;
        end
        
        function theta = getTheta(obj)
            theta = obj.theta;
        end
        
        function v = getV(obj)
            v = obj.v;
        end
        
        function w = getW(obj)
            w = obj.w;
        end
    end
end