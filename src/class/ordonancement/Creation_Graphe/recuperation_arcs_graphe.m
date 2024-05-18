fic = fopen(['Topologique_VF/mon_fichier.txt'], 'r');
nodes = points;

% Vérifier si le fichier est ouvert avec succès
if fic == -1
    error('Impossible d''ouvrir le fichier.');
end

% Itérer sur les lignes du fichier
line = fgetl(fic);
edges = []
id = 1
while ischar(line)
    tab = strsplit(line, ',');
    dep = str2double(tab(1:2));
    arr = str2double(tab(3:4));

    weight = str2double(tab(5))

    id_dep = recherche_dans_points(dep(1),dep(2),nodes);
    id_arr = recherche_dans_points(arr(1),arr(2),nodes);
    if(~isempty(id_dep) && ~isempty(id_arr))
        edges = [edges;[id_dep,id_arr,weight,id]];
        id = id + 1;
    else
        error("Points inconnu");
    end
    % Lire la ligne suivante
    line = fgetl(fic);
end

% Fermer le fichier
fclose(fic);

