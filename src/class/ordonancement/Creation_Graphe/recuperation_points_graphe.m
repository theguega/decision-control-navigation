fic = fopen('Topologique_VF/mon_fichier.txt', 'r');

% Vérifier si le fichier est ouvert avec succès
if fic == -1
    error('Impossible d''ouvrir le fichier.');
end

% Itérer sur les lignes du fichier
line = fgetl(fic);
points = [];
id = 1;
while ischar(line)
    % Séparer la ligne en coordonnées X et Y
    tab = strsplit(line, ',');
    point1 = str2double(tab(1:2));
    point2 = str2double(tab(3:4));
    
    % Vérifier si les points sont déjà dans le tableau
    index_point1 = recherche_dans_points(point1(1), point1(2), points);
    index_point2 = recherche_dans_points(point2(1), point2(2), points);
    
    % Ajouter les points s'ils ne sont pas déjà dans le tableau
    if isnan(index_point1)
        points = [points; point1, id,1];
        id = id + 1;
    end

    if isnan(index_point2)
        points = [points; point2, id,1];
        id = id + 1;
    end
    points(:,4)=1;
    
    % Lire la ligne suivante
    line = fgetl(fic);
end

% Fermer le fichier
fclose(fic);

