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
    end
    
    methods
        
        function obj = vehicle(x, y, theta, v, w)
            obj.x = x;
            obj.y = y;
            obj.theta = theta;
            obj.v = v;
            obj.w = w;
        end
        
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