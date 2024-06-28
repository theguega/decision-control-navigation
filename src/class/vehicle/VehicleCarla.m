classdef VehicleCarla < vehicle
    properties
        CarlaVehicle
        Simulator
        isDeclared
        isEngaged
    end
    methods
        function obj = VehicleCarla(simulator, obstacles, nb_cars_since_beginning,id_road,isEgo)
            point0 = simulator.MapDetail.Map(string(id_road)).waypoints{1};
            point1 = simulator.MapDetail.Map(string(id_road)).waypoints{2};
            pos0= point0.transform.location;
            pos1= point1.transform.location;
            theta = atan2(-pos1.y + pos0.y, pos1.x - pos0.x);
            obj@vehicle(pos0.x, -pos0.y, theta, 0, obstacles, repmat(target(0,0,0,0,0), 0, 0), [], nb_cars_since_beginning, id_road, 0);
            obj.CarlaVehicle = Vehicle(simulator,isEgo);
            obj.Simulator = simulator;
            obj.CarlaVehicle.setPosAndHeading(obj.x, obj.y, obj.theta);
            obj.addTargetRoad(id_road);
            obj.isDeclared=false;
            obj.isEngaged=false;
        end
        function teleportToFirstTarget(obj)
            % Call obj after adding targets to the vehicle to sync the
            % "decision" vehicle and the CARLA vehicle's position and angle
            obj.x = obj.targets(1).x;
            obj.y = obj.targets(1).y;
            obj.theta = obj.targets(1).theta;
            obj.CarlaVehicle.setPosAndHeading(obj.x, obj.y, obj.theta);
        end
        function update(obj, dt, sched)
            obj.actualPath.cost = obj.updatecost();
            if nargin <= 2
                sched = NaN;
            end
            if length(obj.targets)<=2
                if ~isnan(obj.actualPath) & ~isempty(obj.actualPath.roads)
                    if obj.isEngaged==true
                        road = obj.Simulator.MapDetail.Map(string(obj.id_road));
                        obj.isEngaged=false;
                        obj.isDeclared=false;
                        obj.Simulator.MapDetail.Junctions(string(road.junctionId))=obj.Simulator.MapDetail.Junctions(string(road.junctionId))-10;
                        obj.id_road=obj.actualPath.roads(1);
                        obj.actualPath.roads= obj.actualPath.roads(2:end);
                        obj.addTargetRoad(obj.id_road);
                    else
                        nextroad = obj.actualPath.roads(1);
                        road = obj.Simulator.MapDetail.Map(string(nextroad));
                        if road.junctionId>0
                            if isKey(obj.Simulator.MapDetail.Junctions, string(road.junctionId))==false
                                obj.Simulator.MapDetail.Junctions(string(road.junctionId))= 0;
                            end
                            if road.signType == "206" %stop sign
                                if obj.isDeclared==false
                                    obj.Simulator.MapDetail.Junctions(string(road.junctionId))=obj.Simulator.MapDetail.Junctions(string(road.junctionId))+1;
                                    obj.isDeclared=true;
                                end
                                if obj.Simulator.MapDetail.Junctions(string(road.junctionId))<=1
                                    obj.Simulator.MapDetail.Junctions(string(road.junctionId))=obj.Simulator.MapDetail.Junctions(string(road.junctionId))+9;
                                    
                                    obj.id_road=obj.actualPath.roads(1);
                                    obj.actualPath.roads= obj.actualPath.roads(2:end);
                                    obj.addTargetRoad(obj.id_road);
                                    obj.isEngaged=true;
                                end
                            elseif road.signType == "205" %yield sign
                                if obj.isDeclared==false
                                    obj.Simulator.MapDetail.Junctions(string(road.junctionId))=obj.Simulator.MapDetail.Junctions(string(road.junctionId))+3;
                                    obj.isDeclared=true;
                                end
                                if obj.Simulator.MapDetail.Junctions(string(road.junctionId))<=4
                                        obj.Simulator.MapDetail.Junctions(string(road.junctionId))=obj.Simulator.MapDetail.Junctions(string(road.junctionId))+7;
                                        obj.id_road=obj.actualPath.roads(1);
                                        obj.actualPath.roads= obj.actualPath.roads(2:end);
                                        obj.addTargetRoad(obj.id_road);
                                        obj.isEngaged=true;
                                end

                            else
                               if (road.junctionId~=149 & road.junctionId~=59) | (obj.CarlaVehicle.Actor.get_traffic_light_state().name == "Green" | obj.CarlaVehicle.Actor.get_traffic_light_state().name == "Unknown" | obj.CarlaVehicle.Actor.get_traffic_light_state().name == "Off")
                                if obj.isDeclared==false
                                    obj.Simulator.MapDetail.Junctions(string(road.junctionId))=obj.Simulator.MapDetail.Junctions(string(road.junctionId))+5;
                                    obj.isDeclared=true;
                                end
                                if obj.Simulator.MapDetail.Junctions(string(road.junctionId))<=14
                                        obj.Simulator.MapDetail.Junctions(string(road.junctionId))=obj.Simulator.MapDetail.Junctions(string(road.junctionId))+5;
                                        obj.id_road=obj.actualPath.roads(1);
                                        obj.actualPath.roads= obj.actualPath.roads(2:end);
                                        obj.addTargetRoad(obj.id_road);
                                        obj.isEngaged=true;
                                end
                               end
                            end
                        else
                            obj.id_road=obj.actualPath.roads(1);
                            obj.actualPath.roads= obj.actualPath.roads(2:end);
                            obj.addTargetRoad(obj.id_road);
                        end
                    end
                end
            end
            update@vehicle(obj, dt, sched);
            obj.CarlaVehicle.setPosAndHeading(obj.x, obj.y, obj.theta);
        end
        function addTargetRoad(obj, roadId)
            roadList = obj.Simulator.MapDetail.Map(string(roadId)).waypoints;
            startWaypoint = 1;

            if isempty(obj.targets)
                t0x = roadList{1}.transform.location.x;
                t0y = -roadList{1}.transform.location.y;
                t1x = roadList{2}.transform.location.x;
                t1y = -roadList{2}.transform.location.y;
                theta_target = atan2(t1y - t0y, t1x - t0x);
                obj.targets(1) = target(t0x, t0y, theta_target, 0, 0);
                startWaypoint = 2;
            end

            for i = startWaypoint:length(roadList)-1
                road = roadList{i};
                t0x = obj.targets(end).x;
                t0y = obj.targets(end).y;
                t1x = road.transform.location.x;
                t1y = -road.transform.location.y;
                theta_target = atan2(t1y - t0y, t1x - t0x);
                obj.targets(end+1) = target(t1x, t1y, theta_target, 30/3.6, 0);
            end
        end

        function cost = updatecost(obj)
            cost=0;
            for i=1:length(obj.actualPath.roads)
                id_road = obj.actualPath.roads(i);
                road = obj.Simulator.MapDetail.Map(string(id_road));
                cost = cost+road.lengthMeters;
            end
        end
    end
end