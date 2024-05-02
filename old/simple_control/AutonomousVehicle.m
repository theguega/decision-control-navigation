classdef AutonomousVehicle < handle
    properties
        x       % Position in x-axis
        y       % Position in y-axis
        v       % Velocity
        theta   % Orientation angle
    end
    
    methods
        function obj = AutonomousVehicle(x, y, v, theta)
            if nargin == 4
                obj.x = x;
                obj.y = y;
                obj.v = v;
                obj.theta = theta;
            else
                error('Incorrect number of arguments for AutonomousVehicle constructor');
            end
        end
        
        function moveToPoint(obj, target_x, target_y, cruising_speed, arrival_speed, ax)
            speed_text = text(0.1, 0.9, '', 'Units', 'normalized', 'FontSize', 12);
            % PID controller parameters
            kp_v = 1;
            ki_v = 0.1;
            kd_v = 0.01;
            kp_theta = 1;
            ki_theta = 0.1;
            kd_theta = 0.01;
            
            % Error for velocity PID controller
            v_error_prev = 0;
            v_integral = 0;
            
            % Error for angle PID controller
            theta_error_prev = 0;
            theta_integral = 0;
            
            % Simulation parameters
            dt = 0.1;

            % Initialize plot handles
            positionHandle = [];
            velocityHandle = [];
            
            distance = norm([obj.x - target_x, obj.y - target_y]);
            while distance > 0.1
                speed_text.String = ['Current Speed: ', num2str(obj.v)];
                % Calculate desired angle towards target point
                desired_theta = atan2(target_y - obj.y, target_x - obj.x);
                theta_error = desired_theta - obj.theta;
        
                % Control using PID
                theta_integral = theta_integral + theta_error * dt;
                theta_derivative = (theta_error - theta_error_prev) / dt;
                theta_w = kp_theta * theta_error + ki_theta * theta_integral + kd_theta * theta_derivative;
                obj.theta = obj.theta + theta_w * dt;
        
                % Calculate desired speed between target point and crusing speed
                desired_speed = cruising_speed;
                if distance < 3
                    desired_speed = arrival_speed;
                end
                v_error = desired_speed - obj.v;

                % Control using PID
                v_integral = v_integral + v_error * dt;
                v_derivative = (v_error - v_error_prev) / dt;
                v_control = kp_v * v_error + ki_v * v_integral + kd_v * v_derivative;
                obj.v = obj.v + v_control * dt;
        
                % Update position based on velocity and angle
                obj.x = obj.x + obj.v * cos(obj.theta) * dt;
                obj.y = obj.y + obj.v * sin(obj.theta) * dt;
                
                % Recalculate distance to target point
                distance = norm([obj.x - target_x, obj.y - target_y]);
                
                % Update previous errors for theta and velocity
                theta_error_prev = theta_error;
                v_error_prev = v_error;
                
                % Remove previous points
                if ~isempty(positionHandle) && ishandle(positionHandle)
                    delete(positionHandle);
                end
                if ~isempty(velocityHandle) && ishandle(velocityHandle)
                    delete(velocityHandle);
                end

                % Plot vehicle pos and velocuty vector
                positionHandle = plot(ax, obj.x, obj.y, 'bo', 'MarkerSize', 10, 'LineWidth', 2);
                velocityHandle = quiver(ax, obj.x, obj.y, obj.v * cos(obj.theta), obj.v * sin(obj.theta), 'r');
                drawnow;
                if obj.v~=0
                    pause_duration = 1 / 8*(1 + obj.v);
                    pause(pause_duration);
                end
                
            end
        
            % Update final coords
            obj.x = target_x;
            obj.y = target_y;

            delete(speed_text);
            delete(positionHandle);
            delete(velocityHandle);
        end
    end
end
