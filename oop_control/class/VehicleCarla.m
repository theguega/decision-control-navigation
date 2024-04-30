classdef VehicleCarla < vehicle
    properties
        CarlaVehicle
        Simulator
    end
    methods
        function this = VehicleCarla(simulator, x, y, theta, obstacles)
            this@vehicle(x, y, theta, obstacles, []);
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
            roadTuple = this.Simulator.MapParse.waypoint_tuple_list(roadId);
            roadStart = roadTuple{1}{1};
            roadList = roadStart.next_until_lane_end(1);

            for i = 1:length(roadList)
                road = roadList{i};
                this.targets = [this.targets; road.transform.location.x -road.transform.location.y rand() * 3];
            end
        end
    end
end