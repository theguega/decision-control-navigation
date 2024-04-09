classdef obstacle < handle
    properties
        x=0
        y=0
        r=1
        
        marge=0 %marge de sécurité
    end
    methods
        function obj = obstacle(x,y,r,marge)
            obj.x = x;
            obj.y = y;
            obj.r = r;
            obj.marge = marge;
        end
        
        function x = getX(obj)
            x = obj.x;
        end
        
        function y = getY(obj)
            y = obj.y;
        end
        
        function r = getRayonInfluence(obj)
            r = obj.r + obj.marge;
        end
        
        function plot(obj)
            %obstacle
            th = 0:0.01:2*pi;
            xobst = obj.r * cos(th) + obj.x;
            yobst = obj.r * sin(th) + obj.y;
            
            %cercle de sécurité
            xsecu = (obj.r + obj.marge) * cos(th) + obj.x;
            ysecu = (obj.r + obj.marge) * sin(th) + obj.y;
            plot(obj.x,obj.y,'k+','Color','b','LineWidth',1)
            plot(xobst,yobst,'k','Color','b','LineWidth',2)
            plot(xsecu,ysecu,'k:','Color','b','LineWidth',2)
        end
    end
end