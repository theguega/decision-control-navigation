function [NavG] = repair_nav(NavG,pos_to_rm,pos_instead)
    % Problème dans le graph fourni, permet de remplacer un node par un
    % autre dans les liens et le suprime  dans le states
    for i = size(NavG.Links.EndStates,1):-1:1
        if NavG.Links.EndStates(i,1)==pos_to_rm
            tmp = NavG.Links.EndStates(i,2);
            weight = NavG.Links.Weights(i);
            id_road = NavG.Links.Id_route(i);
            rmlink(NavG,[NavG.Links.EndStates(i,1) NavG.Links.EndStates(i,2)]);
            disp(i);
            addlink(NavG,[pos_instead tmp],weight,id_road);
        end
        if NavG.Links.EndStates(i,2)==pos_to_rm
            tmp = NavG.Links.EndStates(i,1);
            weight = NavG.Links.Weights(i);
            id_road = NavG.Links.Id_route(i);
            rmlink(NavG,[NavG.Links.EndStates(i,1) NavG.Links.EndStates(i,2)]);
            disp(i);
            addlink(NavG,[tmp pos_instead],weight,id_road);
        end
    end
    rmstate(NavG,pos_to_rm);



end