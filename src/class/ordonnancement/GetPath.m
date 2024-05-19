function [pathOutput,solutionInfo] = GetPath(navG, id_dep,id_arr)
    % Prend en entrée le NavGraph ainsi que les positions des points de départ et
    % d'arrivée
    % Le path output contient les coordonnées des points de départ des routes à
    % suivre.
    planner =  plannerAStar(navG);
    
    [pathOutput,solutionInfo]=plan(planner,id_dep,id_arr);

end