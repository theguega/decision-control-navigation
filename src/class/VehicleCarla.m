classdef VehicleCarla < vehicle
    properties
        CarlaVehicle
        Simulator
    end
    methods
        function this = VehicleCarla(simulator, x, y, theta, obstacles)
            this@vehicle(x, y, theta, 0, obstacles, repmat(target(0,0,0,0,0), 0, 0));
            this.CarlaVehicle = Vehicle(simulator);
            this.CarlaVehicle.setPos(x, y);
            this.CarlaVehicle.setHeading(theta);
            this.Simulator = simulator;
        end
        function update(this, dt)
            update@vehicle(this, dt);
            this.CarlaVehicle.setPos(this.x, this.y);
            this.CarlaVehicle.setHeading(this.theta);
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