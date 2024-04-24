classdef VehicleCarla < vehicle
    properties
        CarlaVehicle
    end
    methods
        function this = VehicleCarla(simulator, x, y, theta, v, w, obstacles, targets)
            this@vehicle(x, y, theta, v, w, obstacles, targets);
            this.CarlaVehicle = Vehicle(simulator);
            this.CarlaVehicle.setPos(x, y);
            this.CarlaVehicle.setHeading(theta);
        end
        function update(this, dt)
            update@vehicle(this, dt);
            this.CarlaVehicle.setPos(this.x, this.y);
            this.CarlaVehicle.setHeading(this.theta);
        end
    end
end