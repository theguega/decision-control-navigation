function [] = show_path(nav,pathOutput,solutionInfo)
    h = show(nav);
    set(h,XData=nav.States.StateVector(:,1), ...
          YData=nav.States.StateVector(:,2))
    pathStateIDs = solutionInfo.PathStateIDs;
    highlight(h,pathStateIDs,EdgeColor="#EDB120",LineWidth=4)
    highlight(h,pathStateIDs(1),NodeColor="#77AC30",MarkerSize=5)
    highlight(h,pathStateIDs(end),NodeColor="#D95319",MarkerSize=5)
end