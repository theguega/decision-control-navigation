function [pos] = get_pos_by_id(NavG,id)
    % Renvoie la position du noeud recheché en passant son id 
    pos = find(NavG.States.Id_point == id);
end