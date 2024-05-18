classdef VehicleCarla < vehicle
    properties
        CarlaVehicle
        Simulator
    end
    methods
        function this = VehicleCarla(simulator, obstacles, id_vehicle,id_road)
            disp(id_road);
            point0 = simulator.MapDetail.Map(string(id_road)).waypoints{1};
            point1 = simulator.MapDetail.Map(string(id_road)).waypoints{2};
            pos0= point0.transform.location;
            pos1= point1.transform.location;
            theta = atan2(-pos1.y + pos0.y, pos1.x - pos0.x);
            this@vehicle(pos0.x, pos0.y, theta, 0, obstacles, repmat(target(0,0,0,0,0), 0, 0), [], id_vehicle, id_road, 0);
            this.CarlaVehicle = Vehicle(simulator);
            this.Simulator = simulator;
            this.CarlaVehicle.setPosAndHeading(this.x, -this.y, this.theta);
        end
        function teleportToFirstTarget(this)
            % Call this after adding targets to the vehicle to sync the
            % "decision" vehicle and the CARLA vehicle's position and angle
            this.x = this.targets(1).x;
            this.y = this.targets(1).y;
            this.theta = this.targets(1).theta;
            this.CarlaVehicle.setPosAndHeading(this.x, this.y, this.theta);
        end
        function update(this, dt)
            if length(this.targets)<=2
                if ~isempty(this.actualPath.roads)
                    this.id_road=this.actualPath.roads(1);
                    this.actualPath.roads= this.actualPath.roads(2:end);
                    this.addTargetRoad(this.id_road);
                end
            else
                update@vehicle(this, dt);
                this.CarlaVehicle.setPosAndHeading(this.x, this.y, this.theta);
            end
        end
        function addTargetRoad(this, roadId)
            roadList = this.Simulator.MapDetail.Map(string(roadId)).waypoints;

            for i = 1:length(roadList)-1
                road = roadList{i};
                if isempty(this.targets)
                    t0x = road.transform.location.x;
                    t0y = -road.transform.location.y;
                    t1x = roadList{i+1}.transform.location.x;
                    t1y = -roadList{i+1}.transform.location.y;
                else
                    t0x = this.targets(end).x;
                    t0y = this.targets(end).y;
                    t1x = road.transform.location.x;
                    t1y = -road.transform.location.y;
                end
                theta_target = atan2(t1y - t0y, t1x - t0x);
                if isempty(this.targets)
                    this.targets(end+1) = target(t0x, t0y, theta_target, 0, 0);
                else
                    this.targets(end+1) = target(t1x, t1y, theta_target, 0, 0);
                end
                this.targets(end).plot();
            end
        end
    end
end