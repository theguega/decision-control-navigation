function valid_combs = getComb(assignatedDemands,newDemand)
    var = [];
    for i = 1:length(assignatedDemands)
        var = [var;assignatedDemands(i).id_dep];
        var = [var;assignatedDemands(i).id_arr];
    end
    var = [var;newDemand.id_dep];
    var = [var;newDemand.id_arr];
    nb = length(var);
    combs = perms(var);
    invalid_pairs = [];
    for j = 1:length(assignatedDemands)
        invalid_pairs = [invalid_pairs; assignatedDemands(j).id_arr,assignatedDemands(j).id_dep];
    end
    invalid_pairs = [invalid_pairs;newDemand.id_arr,newDemand.id_dep];
    valid_combs = [];
    
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
            valid_combs = [valid_combs; combs(ind,:)];
        end
    end
end