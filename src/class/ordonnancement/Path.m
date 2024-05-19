classdef Path < handle

    properties
        points;
        roads;
        cost;
        Id_vehicle;
    end

    methods
        function obj = Path(path,cost,NavG,id)
            obj.points = path;
            obj.roads = GetRoadsFromPath(path,NavG);
            obj.cost = cost;
            obj.Id_vehicle = id;
        end

        function [points] = getPoints(obj)
            points = obj.points;
        end
        
        function tf = isnan(obj)
            tf = any(isnan([obj.cost]));
        end
    end
end