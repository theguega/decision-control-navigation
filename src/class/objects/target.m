classdef target < handle
    properties
        x;
        y;
        theta;
        v;
        w;
    end
    methods
        function obj = target(x, y, theta, v, w)
            obj.x = x;
            obj.y = y;
            obj.theta = theta;
            obj.v = v;
            obj.w=w;
        end

        function ct = getCurv(obj)
            if obj.w~=0
                ct = obj.v/obj.w;
            else
                ct=0;
            end
        end
        
        function plot(obj)
            %plot the target and its orientation
            plot(obj.x, obj.y, 'k+', 'LineWidth', 2);
            speed = 1;
            if obj.v~=0
                speed=obj.v;
            end
            plot([obj.x, obj.x + cos(obj.theta) * 3 * speed ], [obj.y, obj.y + sin(obj.theta) * 3 * speed]);
        end

        function update(obj, dt)
            obj.x = obj.x + obj.v * cos(obj.theta) * dt;
            obj.y = obj.y + obj.v * sin(obj.theta) * dt;
            obj.theta = obj.theta + obj.w * dt;
        end
            
    end
end