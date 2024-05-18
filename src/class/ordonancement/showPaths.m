function [] = showPaths(path1,path2,NavG)
    h = show(NavG);
    set(h,XData=NavG.States.StateVector(:,1), ...
        YData=NavG.States.StateVector(:,2));
    pathStateIDs = [];
    for i = 1:size(path1.points,1)
        pathStateIDs = [pathStateIDs;get_pos_by_id(NavG,recherche_dans_nodes(path1.points(i,1),path1.points(i,2),NavG))];
    end
    highlight(h,pathStateIDs,EdgeColor="#EDB120",LineWidth=4);
    highlight(h,pathStateIDs(1),NodeColor="#77AC30",MarkerSize=5);
    highlight(h,pathStateIDs(end),NodeColor="#D95319",MarkerSize=5);
    pathStateIDs2 = [];
    for i = 1:size(path2.points,1)
        pathStateIDs2 = [pathStateIDs2;get_pos_by_id(NavG,recherche_dans_nodes(path2.points(i,1),path2.points(i,2),NavG))];
    end
    highlight(h,pathStateIDs2,EdgeColor="#EDB120",LineWidth=4);
    highlight(h,pathStateIDs2(1),NodeColor="#77AC30",MarkerSize=5);
    highlight(h,pathStateIDs2(end),NodeColor="#D95319",MarkerSize=5);
end