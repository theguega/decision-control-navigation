function [id] = get_id_by_pos(NavG,pos)
    % Renvoie l'id du noeud recheché en passant sa position
    id = NavG.States.Id_point(pos);
end