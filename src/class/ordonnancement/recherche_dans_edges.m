function [id_route] = recherche_dans_edges(id_dep,id_arr,navG)
    id_route = NaN;
    for i = 1:size(navG.Links,1)
        if(navG.Links.EndStates(i,1)==id_dep && navG.Links.EndStates(i,2)==id_arr)
            id_route = navG.Links.Id_route(i);
            return
        end
    end
end