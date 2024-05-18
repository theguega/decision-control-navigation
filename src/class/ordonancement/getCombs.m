function valid_combs = getCombs(id_dep,demands)
    % renvoie l'ensemble des combinaisons possible dans le cas d'un
    % regroupement de trajets, id_dep correspond à la position actuelle du
    % véhicule


    var=[];
    full_demands = [];
    

    % ajout id pour recherches de toutes les permutations possibles (sauf
    % l'id_dep car forcément le premier doinc ajouté après)

    for i = 1:size(demands,1)
        if demands(i).id_dep ~= id_dep
            % si le point de départ d'une demande coincide avec le point de
            % départ du véhicule, pas besoin d'ajouter l'id car il sera
            % forcément le premier (ajouté après le calcul des combinaisons
            % possible)
            if demands(i).visited_dep == false
                var = [var;demands(i).id_dep];
                full_demands = [full_demands;demands(i)];
            end
        end
        
        var = [var;demands(i).id_arr];
    end


    nb = length(var);
    combs = perms(var); % calcul de toutes les permutations
    invalid_pairs = [];
    
    % création de toutes les contraintes 
    for j = 1:length(full_demands)
        invalid_pairs = [invalid_pairs; full_demands(j).id_arr,full_demands(j).id_dep];
    end
    valid_combs = [];
    
    % supression des combinaisons ne validant pas les contraintes
    for ind = 1:size(combs, 1)
        valid = true;
        for i = 1:nb
            for j = i+1:nb
                
                if ismember([combs(ind,i),combs(ind,j)], invalid_pairs, 'rows') || (combs(ind,1) ~= var(1))
                    valid = false;
                    break;
                end
            end
            if ~valid
                break;
            end
        end
        if valid
            valid_combs = [valid_combs; [id_dep,combs(ind,:)]]; % ajout de l'id de départ au début de la combinaison
        end
    end
end
