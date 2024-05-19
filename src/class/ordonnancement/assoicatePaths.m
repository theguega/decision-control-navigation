function [associated_path] = assoicatePaths(paths,total_cost,NavG,id_vehicle)
    % Prend en entrée une table de Path dont les départs coincident avec
    % les arrivées des suivants et renvoie le path global
    if isempty(paths)
        error("Votre tableau est vide");
    end
    global_path = paths{1,1}.points;
    for i = 2:size(paths,1)
        points = paths{i,1}.points;
        global_path = [global_path;points(2:size(points,1),:)];
    end
    associated_path = Path(global_path,sum(total_cost),NavG,id_vehicle);
    
end