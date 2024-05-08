classdef target < handle
    properties
        x;
        y;
        theta;
        target_speed;
    end
    methods
        function obj = target(x, y, target_speed, theta)
            obj.x = x;
            obj.y = y;
            obj.theta = theta;
            obj.target_speed = target_speed;
        end
        
        function x=getX(obj)
            x = obj.x;
        end
        
        function y=getY(obj)
            y = obj.y;
        end
        
        function theta=getTheta(obj)
            theta = obj.theta;
        end
        
        function s = getTargetSpeed(obj)
            s = obj.target_speed;
        end
        
        function plot(obj)
            %plot the target and its orientation
            plot(obj.x, obj.y, 'k+', 'LineWidth', 2);
            plot([obj.x, obj.x + cos(obj.theta) * 3 ], [obj.y, obj.y + sin(obj.theta) * 3]);
        end
    end
end