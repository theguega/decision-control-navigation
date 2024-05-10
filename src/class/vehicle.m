classdef vehicle < handle
    properties
        x = 0;
        y = 0;
        theta = 0;
        gamma = 0;
        v = 0; %linear speed

        id = 0;
        lbase = 3;

        %plot parameters
        L = 2.5;     % Empattement
        l = 3;  % Largeur des essieux
        r = 1;     % Rayon des roues
        
        dt = 0.1; % discretisation time
        
        %targets and obstacles
        targets = []; %(x,y,speed_target)
        obstacles = []; %obstacle object
        actual_target;
        
        %controllers parameters
        RayonCycleLimite=0;
        controller = 1;

        %controllers memory
        to_avoid = obstacle(0,0,0,0); %obstacle to avoid
        distance_to_avoid=-1; % distance between nearest obstacle and the vehicle
        v_error_prev = 0; %velocity error for PID controller
        v_integral = 0;

        %arrays for visualisation
        theta_error_output=[];
        speed_error_output=[];
    end
    
    methods
        %constructor
        function obj = vehicle(x, y, theta, gamma, obstacles, targets)
            if nargin == 6
                obj.x = x;
                obj.y = y;
                obj.theta = theta;
                obj.gamma = gamma;
                obj.obstacles = obstacles;

                %treatment of targets : calculate target speed based on
                %waypoint navigation method + delete waypoints that are on
                %static targets
                obj.targets = targets;
            else
                 error('Incorrect number of arguments for vehicle constructor');
            end
        end
        
        %plot the vehicle pos
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
            obj.dt = dt;
            
            t = [obj.targets(1).getTheta, obj.targets(1).getX, obj.targets(1).getY];
            T_O_T = [cos(t(1)) -sin(t(1)) t(2)
                     sin(t(1)) cos(t(1))  t(3)
                     0         0          1];

            vehicle_base_target = T_O_T\[obj.getX; obj.getY; 1];
            
            x_vehicle_base_target = vehicle_base_target(1);
            error_distance = obj.get_distance_point(obj.targets(1));
            error_theta = abs(obj.theta-obj.targets(1).getTheta);
            
            % check if the point is already reached
            if ( ((error_theta<=1.5) && (error_distance<=1)) || (x_vehicle_base_target>0))
                obj.targets(1)=[];
            end
            
            if ~isempty(obj.targets)
                obj.actual_target=obj.targets(1); % first to reach
                control = obj.controller_selection(); % return controller command
                if obj.controller==1
                    obj.set_pos_attraction(control);
                elseif obj.controller==2
                    obj.set_pos_avoidance(control);
                end
            end
        end
        
        
        
        
        %choose the right controller depending on the situation and give the correponding output
        function control=controller_selection(obj)
            %reset obstacle to avoid
            obj.to_avoid = obstacle(0,0,0,0);
            obj.distance_to_avoid=-1;
            obj.controller=1; % attraction by default
            
            %check if there is an obstacle to avoid
            for i=1:size(obj.obstacles,2)
                dist = sqrt( (obj.obstacles(i).getX() - obj.x)^2 + (obj.obstacles(i).getY() - obj.y)^2);
                if dist<=(obj.obstacles(i).getRayonInfluence())
                    obj.distance_to_avoid=dist;
                    %if the actual obstacle is neareast than the previous one
                    if dist<=obj.distance_to_avoid
                        obj.to_avoid=obj.obstacles(i);
                        obj.distance_to_avoid=dist;
                        obj.controller=2;
                    end
                end
            end
            
            % get appropriates data depending on controller type
            if obj.controller==2
                datas=obj.var_obstacle_avoidance();
                control=obj.control_obstacle_avoidance(datas);
            else
                datas=obj.var_attraction();
                control=obj.control_attraction(datas);
            end
        end
        
        function datas=var_attraction(obj)
            %compute the datas for the attraction controller
            xe = obj.actual_target.getX - obj.x;
            ye = obj.actual_target.getY - obj.y;
            error_x = cos(obj.theta)*xe + sin(obj.theta)*ye;
            error_y = -sin(obj.theta)*xe + cos(obj.theta)*ye;
            
            error_theta = subtr_ang(obj.actual_target.getTheta, obj.theta);
            
            thetaRT = atan2(ye,xe);
            error_RT = subtr_ang(obj.actual_target.getTheta, thetaRT);
            
            datas = [error_x; error_y; error_theta; error_RT];
        end
        
        function control=control_attraction(obj, datas)
            %datas = [error_x; error_y; error_theta; error_RT]
            d=sqrt(datas(1)^2 + datas(2)^2);

            %Controller parameters
            K_d = 10;     %Main for Ex (d in the Lypauniv function)
            K_l = 5;      %Main for "d", error E_RT and E_theta
            K_o = 2;      %Main for E_theta and C_c ATTENTION NE DOIT PAS ETRE EGAL A "0", On divise sur K_o %%0.3;%1;%1.2/distf steering
            K_theta = 5;  %Main for E_Theta 0.8;%1.0
            K_rt = 0.01;  %Main for E_RT %%0.01

            CosE_theta = cos(datas(3));
            SinE_theta = sin(datas(3));

            SinE_RT = sin(datas(4));
            CosE_RT = cos(datas(4));

            Curvature_T = 0; % in case of moving targets
            curv =   Curvature_T/CosE_theta + ... %%1st term
                     (d^2*Curvature_T*K_l*SinE_RT*CosE_RT)/(K_o*SinE_theta*CosE_theta)+ ...%%2sd term
                     K_theta*(SinE_theta/CosE_theta)+ ... %%3rd term
                     (K_d*datas(2) - K_l*d*SinE_RT*CosE_theta)/(K_o*CosE_theta)+... %%4rd term
                     (K_rt*SinE_RT^2)/(SinE_theta*CosE_theta);

            if isnan(curv) %% CASE WHERE THERE IS A PROBLEME OF COMPUTATIONS
                curv = 0.0001;
            end

            gamma_controller = atan(obj.lbase*curv);

            obj.theta_error_output = [obj.theta_error_output;[obj.theta obj.actual_target.getTheta]];

            % PID controller parameters
            kp_v = 1;
            ki_v = 0.1;
            kd_v = 0.01;
            
            %----- control linear speed -----
            target_speed=obj.actual_target.getTargetSpeed;
            obj.speed_error_output = [obj.speed_error_output;[obj.v target_speed]];
            v_error = target_speed - obj.v;
            obj.v_integral = obj.v_integral + v_error * obj.dt;
            v_derivative = (v_error - obj.v_error_prev) / obj.dt;
            v_control = kp_v * v_error + ki_v * obj.v_integral + kd_v * v_derivative;
            V = obj.v + v_control * obj.dt;
            
            control = [V , gamma_controller];
        end
        
        
        
        function datas=var_obstacle_avoidance(obj)
            %compute the datas for the obstacle avoidance controller
            error_x = obj.x - obj.to_avoid.getX();
            error_y = obj.y - obj.to_avoid.getY();
            
            %transformation matrix
            X_D_O = obj.actual_target.getX - obj.to_avoid.getX();
            Y_D_O = obj.actual_target.getY - obj.to_avoid.getY();
            alpha = atan2(Y_D_O, X_D_O);
            T_O_A = [cos(alpha) -sin(alpha) 0 obj.to_avoid.getX()
                     sin(alpha)  cos(alpha) 0 obj.to_avoid.getY()
                     0           0          1 0
                     0           0          0 1];
            
            obstacle_coords = T_O_A\[obj.x; obj.y; 0; 1];
            X_obst = obstacle_coords(1);
            
            % get rayoncyclelimite
            if (X_obst <= 0)
                obj.RayonCycleLimite=(obj.to_avoid.getRayonInfluence()+obj.l)-0.3;
            else
                obj.RayonCycleLimite=obj.RayonCycleLimite+0.03;
            end
            
            x_dot = error_y + error_x * ((obj.RayonCycleLimite^2) - (error_x^2) - (error_y^2));
            y_dot = -error_x + error_y * ((obj.RayonCycleLimite^2) - (error_x^2) - (error_y^2));
            theta_dot = atan2(y_dot, x_dot);
            
            X0 = [x_dot, y_dot];
            xc = ode23(@(t, y) EquationDiff_Tourbillon(t, y, obj.RayonCycleLimite), [0, 0.2], X0);
            theta_controller = atan2((xc.y(2, 2) - xc.y(1, 2)), (xc.y(2, 1) - xc.y(1, 1)));
            error_theta=SoustractionAnglesAtan2(theta_dot, obj.theta);
            
            datas = [error_theta; theta_controller];
        end
        
        
        function control=control_obstacle_avoidance(obj, datas)
            %datas = [error_theta; theta_controller]
            Kp = 15;
            W = datas(2) + Kp*datas(1);

            % PID controller parameters
            kp_v = 1;
            ki_v = 0.1;
            kd_v = 0.01;
            
            %----- control linear speed -----
            target_speed=5/3.6; %fixed to 5km/h in obstacle avoidance mode
            obj.speed_error_output = [obj.speed_error_output;[obj.v target_speed]];
            v_error = target_speed - obj.v;
            obj.v_integral = obj.v_integral + v_error * obj.dt;
            v_derivative = (v_error - obj.v_error_prev) / obj.dt;
            v_control = kp_v * v_error + ki_v * obj.v_integral + kd_v * v_derivative;
            V = obj.v + v_control * obj.dt;

            control = [V, W];
        end
        
        
        function set_pos_attraction(obj, control)
            %update the position of the vehicle
            obj.v = control(1);
            tmp_gamma = control(2);

            obj.x = obj.x + obj.v * cos(obj.theta) * obj.dt;
            obj.y = obj.y + obj.v * sin(obj.theta) * obj.dt;
            obj.theta = obj.theta + (tan(tmp_gamma/obj.lbase)) * obj.v * obj.dt;
            obj.gamma = tmp_gamma;
        end

        function set_pos_avoidance(obj, control)
            %update the position of the vehicle
            obj.v = control(1);
            w = control(2);

            obj.x = obj.x + obj.v * cos(obj.theta) * obj.dt;
            obj.y = obj.y + obj.v * sin(obj.theta) * obj.dt;
            obj.theta = obj.theta + w * obj.dt;
            obj.gamma = obj.theta + w * obj.dt;
        end
        
        
        %---------------------------------------------------- getters ----------------------------------------------------
        
        function dist=get_distance_object(obj, object)
            dist=sqrt((obj.x-object.getX())^2+(obj.y-object.getY())^2); % distance between vehicle and object in arguments
        end
        
        function dist=get_distance_point(obj, target)
            dist=sqrt((obj.x-target.getX)^2+(obj.y-target.getY)^2); %distance between vehcile and point=(x,y)
        end
        
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
        
        function dt = getdt(obj)
            dt = obj.dt;
        end

        function id = getid(obj)
            id = obj.id;
        end

        function gamma = getgamma(obj)
            gamma = obj.gamma;
        end
        
        function obstacles = getobstacles(obj)
            obstacles = obj.obstacles;
        end

        function error = getspeederror(obj)
            error=obj.speed_error_output;
        end

        function error = getthetaerror(obj)
            error=obj.theta_error_output;
        end
    end
end