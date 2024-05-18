classdef scheduler < handle
    % This class contain the scheduling program that assignate cars to
    % demands.

    properties
        users;
        demands;
        cars;
        base_nav; % contain the base navgraph with 
        modified_nav;
        tmp_roads; % table that contains temporary roads created for demands and links them to their mother roads
    end

    methods
        function obj = scheduler(NavG)
            obj.users = [];
            obj.demands = [];
            obj.cars = [];
            obj.base_nav = NavG;
            obj.modified_nav = copy(NavG);
            obj.tmp_roads = table('Size',[0, 3],'VariableTypes',{'double','double','double'},'VariableNames',{'First_tmp_road','Second_tmp_road','Base_road'});

        end

        function obj = createUser(obj,id_user,nom,prenom,id_road,pos_x,pos_y,dist_from_start)
            obj.users = [obj.users; user(id_user,nom,prenom,id_road,pos_x,pos_y,dist_from_start)];
        end

        function car = createCar(obj,simulator, obstacles, id_vehicle,id_road)
            disp(id_road);
            car=VehicleCarla(simulator, obstacles, id_vehicle,id_road);
            obj.cars = [obj.cars; car];
        end

        function obj = addCar(obj,car)
           obj.cars = [obj.cars; car];
        end
        
        function obj = createDemand(obj,Id_road_dep,Id_road_arr,X_coord_dep,Y_coord_dep,dist_to_start_dep,X_coord_arr,Y_coord_arr,dist_to_start_arr,id_user,priority,nbpassengers)
            [id_dep,~,new_r_1,new_r_2] = insertNodeInRoad(obj.modified_nav,Id_road_dep,X_coord_dep,Y_coord_dep,dist_to_start_dep);
            obj.tmp_roads = vertcat(obj.tmp_roads, table(new_r_1, new_r_2, Id_road_dep, 'VariableNames', {'First_tmp_road', 'Second_tmp_road', 'Base_road'}));


        
            [id_arr,~,new_r_1,new_r_2] = insertNodeInRoad(obj.modified_nav,Id_road_arr,X_coord_arr,Y_coord_arr,dist_to_start_arr);
            obj.tmp_roads = vertcat(obj.tmp_roads, table(new_r_1, new_r_2, Id_road_arr, 'VariableNames', {'First_tmp_road', 'Second_tmp_road', 'Base_road'}));
            
            id_dem = 0;
            if ~isempty(obj.demands)
                for i = 1:length(obj.demands)
                    id = obj.demands(i,1).getIdDem;
                    if id>id_dem
                        id_dem = id;
                    end
                end
                id_dem = id_dem+1;
            end
            
            obj.demands = [obj.demands;demand(id_dem,id_dep,id_arr,id_user,priority,nbpassengers)];
        end  

        function obj = allocateDemandToCar(obj,id_dem,id_car)
            % retire la demande du scheduler et l'assigne au véhicule
            % choisi
            pos_dem = NaN;
            for i = 1:size(obj.demands,1)
                if obj.demands(i).id_dem == id_dem
                    pos_dem = i;
                end
            end

            pos_car = NaN;
            for i = 1:size(obj.cars,1)
                if obj.cars(i).id_vehicle == id_car
                    pos_car = i;
                end
            end

            if isnan(pos_dem)||isnan(pos_car)
                error("Unkown id of the demand or of the car inside the scheduler");
            end

            obj.cars(pos_car) = obj.cars(pos_car).addDemand(obj.demands(pos_dem));
            obj.cars(pos_car).addPassenger(obj.demands(pos_dem).nbpassengers);
            obj.cars(pos_car).updatePriority();
            obj.demands = [obj.demands(1:pos_dem-1); obj.demands(pos_dem+1:end)];
        end 

        function obj = schedulDemands(obj)
            % Cette fonction ne tient pas commpte des priorités des
            % demandes. Elle assigne les demandes en temp réel au véhocule
            % pour lequel le détournement sera le plus faible (nouveau cout
            % simulé - cout actuel). 
            % Les assignations se font à mesure que les demandes arrivent.
            % On ne prend que la première demande car elle sont supprimées
            % au fur et à mesure qu'elles sont assignées aux véhicules
            for i = 1:size(obj.demands,1)
                min_deviation_cost = NaN;
                pos_lower_car = NaN;
                best_path = NaN;
                deviation_cost = NaN;
                for j = 1:size(obj.cars)
                    res = NaN;
                    
                    % max 5 demandes et 5 passagers par véhicule
                    if size(obj.cars(j).plannedDemands,1)<5 && 5-obj.cars(j).nbpassengers-obj.demands(1).nbpassengers >= 0
                        dmnds = [obj.cars(j).plannedDemands;obj.demands(1)];
                        res = obj.cars(j).getOptimalPath(dmnds,obj.modified_nav);
                        if isnan(obj.cars(j).actualPath)
                            deviation_cost = res.Cout_total;
                        else
                            deviation_cost = res.Cout_total - obj.cars(j).actualPath.cost;
                        end
                        if isnan(min_deviation_cost)||deviation_cost<min_deviation_cost
                            min_deviation_cost = deviation_cost;
                            pos_lower_car = j;
                            best_path = res.Chemin;

                        end
                    end
                end
                if isnan(pos_lower_car)
                    error("No available vehicle now, try later, or create vehicles");
                end
                obj.cars(pos_lower_car).assignateNewPath(best_path);
                obj.allocateDemandToCar(obj.demands(1).id_dem,obj.cars(pos_lower_car).id_vehicle);


            end
        end
    
        function obj = createRandomDemand(obj,id_user,priority,nbpassenger)
            id = 0;
            for i = 1:size(obj.demands)
                if obj.cars(i).id_vehicle > id
                    id = obj.demands(i).id_dem;
                end
            end
            if id>0 
                id = id +1;
            end
            r_dep = obj.base_nav.Links(randi([1,size(obj.base_nav.Links,1)]),:);
            r_arr = obj.base_nav.Links(randi([1,size(obj.base_nav.Links,1)]),:);
            while r_arr == r_dep
                r_arr = obj.base_nav.Links(rand(1,size(obj.base_nav.Links,1)),:);
            end

            % Paramètres pour la première route
            id_r_dep = r_dep(1,3);
            dep_cut = rand;
            dist_to_start_dep = dep_cut*r_dep(2);
            % Coordonnées du nouveau point
            [x_dep,y_dep] = obj.base_nav.States.StateVector(r_dep(1,1));
            [x_arr,y_arr] = obj.base_nav.States.StateVector(r_dep(1,2));
            depl_x = x_arr - x_dep;
            depl_y = y_arr - y_dep;
            new_x_dep = x_dep + dep_cut*depl_x;
            new_y_dep = y_dep + dep_cut*depl_y;


            % Paramètres pour la seconde route
            id_r_arr = r_arr(1,3);
            arr_cut = rand;
            dist_to_start_arr = dep_cut*r_arr(2);
            % Coordonnées du nouveau point
            [x_dep,y_dep] = obj.base_nav.States.StateVector(r_dep(2,1));
            [x_arr,y_arr] = obj.base_nav.States.StateVector(r_dep(2,2));
            depl_x = x_arr - x_dep;
            depl_y = y_arr - y_dep;
            new_x_arr = x_dep + arr_cut*depl_x;
            new_y_arr = y_dep + arr_cut*depl_y;


            obj.createDemand(id_r_dep,id_r_arr,new_x_dep,new_x_arr,dist_to_start_dep,new_x_arr,new_y_arr,dist_to_start_arr,id_user,priority,nbpassenger);

        
        end
    end
end