function [roads] = GetRoadsFromPath(pathOutput,navG)
% Renvoie la liste des ids des routes à emprunter issue des points fournis par la
% fonction getPath
    points = [];
    for i = 1:size(pathOutput,1)
        point = recherche_dans_nodes(pathOutput(i,1),pathOutput(i,2),navG);
        if ~isnan(point)
            points = [points, point];
        end
    end
    roads=[];
    for j = 1:(size(pathOutput,1)-1)
        road = recherche_dans_edges(points(j),points(j+1),navG);
        if ~isnan(road)
            roads = [roads;road];
        end
    end
end