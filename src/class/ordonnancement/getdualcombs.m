function dual_combs = getdualcombs(combs)
    dual_combs = [];
    for k = 1:size(combs, 1)
        for i = 1:size(combs, 2)-1
            c = [combs(k,i), combs(k,i+1)];
            if isempty(dual_combs)
                dual_combs = [dual_combs; c];
            elseif ~ismember(c, dual_combs, 'rows')
                dual_combs = [dual_combs; c];
            end
            
        end
    end
end


