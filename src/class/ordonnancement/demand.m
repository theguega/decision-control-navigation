classdef demand < handle
    properties
        id_dem;
        id_dep;
        id_arr;
        id_user;
        priority;
        nbpassengers;
        visited_dep; % booléen vrai si les passagers ont déjà été récupérés au point de départ
    end

    methods
        function obj = demand(id_dem,id_dep,id_arr,id_user,prio,nbpassengers)
            if nbpassengers > 5
                error("Number of assenger is limited to 5");
            end
            obj.id_dem = id_dem;
            obj.id_dep = id_dep;
            obj.id_arr = id_arr;
            obj.id_user = id_user;
            obj.priority = prio;
            obj.nbpassengers = nbpassengers;
            obj.visited_dep = false;
        end
        function [id]=getIdDem(obj)
            id = obj.id_dem;
        end
        function [n] = getPrio(obj)
            n = obj.priority;
        end 
        function [tbl_sorted,copied_nav] = getClosestvehicleFromStart(obj,vehicles,NavG)
            % renvoie al liste triée des véhicules disponibles (suffisament
            % de places libres) les plus proches ainsi que le chemin 
            % reliant le véhicule au point de départ de la demande
            paths = []; 
            copied_nav = copy(NavG);

            for i = 1:size(vehicles,1)
                if 5-vehicles(i).nbpassengers-obj.nbpassengers >= 0 % on ne considère que les véhicules qui ont suffisament de place pour traiter la demande
               
                    % insertion du véhicule
                    [~,vcl] = insertNodeInRoad(copied_nav,vehicles(i).getRoad(),vehicles(i).getX(),vehicles(i).getY(),vehicles(i).getDistFromStart());
                    
                    
                    [pathOutput,solutionInfo]=GetPath(copied_nav,vcl,get_pos_by_id(NavG,obj.id_dep));
                    disp(solutionInfo);
                    if solutionInfo.IsPathFound == 1
                        pth = Path(pathOutput,solutionInfo.PathCost,copied_nav,vehicles(i).getIdvehicle());
                        paths = [paths;pth];
                    end
                end
            end    
            disp(paths);
            tbl = table();
            for i = 1:size(paths,1)
                path_cost = paths(i).cost;
                path_id_vehicle = paths(i).Id_vehicle;
                path = paths(i);
                tbl = [tbl; table(path_cost, path_id_vehicle,path)];
            end
            disp(tbl);
            tbl_sorted = sortrows(tbl, 'path_cost');
        end
        
    end
end