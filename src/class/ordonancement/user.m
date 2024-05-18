classdef user < handle
    properties
        id_user;
        nom;
        prenom;
        pos_x;
        pos_y;
        id_road;
        dist_from_start;
    end

    methods
        function obj = user(id_user,nom,prenom,id_road,pos_x,pos_y,dist_from_start)
            obj.id_user = id_user;
            obj.nom = nom;
            obj.prenom = prenom;
            obj.pos_x = pos_x;
            obj.pos_y = pos_y;
            obj.id_road = id_road;
            obj.dist_from_start = dist_from_start;
        end
        

        function [tbl_sorted,copied_nav] = getClosestvehicle(obj,vehicles,NavG)
            paths = []; 
            copied_nav = copy(NavG);
            % insertion de l'utilisateur
            [~,usr] = insertNodeInRoad(copied_nav,obj.id_road,obj.pos_x,obj.pos_y,obj.dist_from_start);
            for i = 1:size(vehicles,1)
                
                % insertion du véhicule
                [~,vcl] = insertNodeInRoad(copied_nav,vehicles(i).getRoad(),vehicles(i).getX(),vehicles(i).getY(),vehicles(i).getDistFromStart());
                
                
                [pathOutput,solutionInfo]=GetPath(copied_nav,vcl,usr);
                % disp(solutionInfo);
                if solutionInfo.IsPathFound == 1
                    pth = Path(pathOutput,solutionInfo.PathCost,copied_nav,vehicles(i).getIdvehicle());
                    paths = [paths;pth];
                end
                
            end    
            tbl = table();
            for i = 1:size(paths,1)
                path_cost = paths(i).cost;
                path_id_vehicle = paths(i).Id_vehicle;
                path = paths(i);
                tbl = [tbl; table(path_cost, path_id_vehicle,path)];
            end
            tbl_sorted = sortrows(tbl, 'path_cost');
        end

        function obj = updatePos(obj,x,y,id_road)
            obj.pos_x = x;
            obj.pos_y = y;
            obj.id_road = id_road;
        end
    end
end