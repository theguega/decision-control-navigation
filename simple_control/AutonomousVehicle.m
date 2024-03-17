classdef AutonomousVehicle < handle
    properties
        x
        y
        theta
        v
        dt
        kp
        ki
        kd
        previous_error
        integral
    end
    
    methods
        function obj = AutonomousVehicle(x, y, theta, v, dt, kp, ki, kd)
            % Conditions initiales
            obj.x = x;
            obj.y = y;
            obj.theta = theta;
            obj.v = v;
            obj.dt = dt;

            % Paramètres PID
            obj.kp = kp;
            obj.ki = ki;
            obj.kd = kd;

            % Errors
            obj.previous_error = 0;
            obj.integral = 0;
        end
        
        function moveToPoint(obj, target_x, target_y, ax)
            % Initialize plot handles
            positionHandle = [];
            velocityHandle = [];
            
            distance = sqrt((target_x - obj.x)^2 + (target_y - obj.y)^2);
            while distance > 0.1
                desired_theta = atan2(target_y - obj.y, target_x - obj.x);
                error = desired_theta - obj.theta;
                obj.integral = obj.integral + error * obj.dt;
                derivative = (error - obj.previous_error) / obj.dt;
                w = obj.kp * error + obj.ki * obj.integral + obj.kd * derivative;
                obj.theta = obj.theta + w * obj.dt;
                obj.x = obj.x + obj.v * cos(obj.theta) * obj.dt;
                obj.y = obj.y + obj.v * sin(obj.theta) * obj.dt;
                distance = sqrt((target_x - obj.x)^2 + (target_y - obj.y)^2);
                obj.previous_error = error;
        
                % Effacer les tracés précédents
                if ~isempty(positionHandle) && ishandle(positionHandle)
                    delete(positionHandle);
                end
                if ~isempty(velocityHandle) && ishandle(velocityHandle)
                    delete(velocityHandle);
                end
        
                % Afficher la position du véhicule
                positionHandle = plot(ax, obj.x, obj.y, 'bo', 'MarkerSize', 10, 'LineWidth', 2);
        
                % Afficher la position du point cible en rouge
                plot(ax, target_x, target_y, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
        
                % Afficher le vecteur vitesse en rouge
                velocityHandle = quiver(ax, obj.x, obj.y, cos(obj.theta), sin(obj.theta), 'r');
        
                % Mettre à jour l'affichage
                drawnow;
        
                % Pause pour visualiser l'animation
                pause(0.1);
            end
        
            % Mettre à jour les coordonnées initiales
            obj.x = target_x;
            obj.y = target_y;
        end
    end
end