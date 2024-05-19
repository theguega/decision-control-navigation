function id = recherche_dans_nodes(x, y, navG)
    id = NaN;
    for i = 1:size(navG.States.StateVector, 1)
        if navG.States.StateVector(i, 1) == x && navG.States.StateVector(i, 2) == y
            id = navG.States.Id_point(i);
            return;
        end
    end
end