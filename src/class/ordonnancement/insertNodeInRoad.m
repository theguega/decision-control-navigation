function [id_node,pos_new_state,id_new_road_1,id_new_road_2] = insertNodeInRoad(NavG,id_road,X,Y,dist_from_start)
    indice_route = find(NavG.Links.Id_route == id_road);
    
    if ~isnan(indice_route)
        id_node = recherche_dans_nodes(X,Y,NavG);
        if isnan(id_node) % Si le noeud n'est pas déjà dans le graphe
            % Création du noeud au mileu de la route
            id_new_node = max(NavG.States.Id_point)+1;

            pos_new_state = addstate(NavG,[X Y],id_new_node,0);
            
            
            id_deb = find(NavG.States.Id_point == NavG.Links.EndStates(indice_route,1));
            
            
            % Création des deux moitiés de la route
            id_new_road_1 = max(NavG.Links.Id_route)+1;
            id_new_road_2 = id_new_road_1+1;
            addlink(NavG,[id_deb pos_new_state],dist_from_start,id_new_road_1); 
            dist_to_arrival = NavG.Links.Weights(indice_route)-dist_from_start;
            id_fin = find(NavG.States.Id_point == NavG.Links.EndStates(indice_route,2)); 
            addlink(NavG,[pos_new_state id_fin],dist_to_arrival,id_new_road_2); 
            
            
            id_node = id_new_node;
            
        else
            pos_new_state = get_pos_by_id(NavG,id_node);
        end
        
    else
        disp("Erreur l'id suivant de route n'existe pas: ");
        disp(id_road);
        error("L'id de la route n'existe pas dans le graphe.");
    end

end