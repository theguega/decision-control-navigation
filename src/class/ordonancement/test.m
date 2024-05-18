
% nodes = load("Topologique_VF/points_topo_VF.mat").points;
% edges = load("Topologique_VF/arcs_topo_vf.mat").edges;

% Obtenir les coordonnées x et y des points
% x_coords = nodes(:, 1);
% y_coords = nodes(:, 2);
% id = nodes(:,3);
%plot(x_coords,y_coords,".")
% for i = 1:size(nodes, 1)
%     text(x_coords(i), y_coords(i), num2str(nodes(i, 3)), 'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');
% end
% xlabel('X');
% ylabel('Y');
% title('Points avec leurs identifiants');
%Visualiser le digraphe avec les coordonnées spécifiques des points
% plot(monDigraph, 'XData', x_coords, 'YData', y_coords);




%state_table = table(nodes(:,1:2),nodes(:,3),nodes(:,4),VariableNames=["StateVector","Id_point","Is_base_node"]);
%link_table = table(edges(:,1:2),edges(:,3),edges(:,4),VariableNames=["EndStates","Weights","Id_route"]);

% nav = navGraph(state_table,link_table)

nav = load("Topologique_VF/nav_topo_vf.mat").nav;




% roads = GetRoadsFromPath(pathOutput,nav);
% insertNodeInRoad(nav,1,2.23936861455827,48.6251716749029,3.344755124419394e-05/2);
% demands = table(0,0,0,0,VariableNames=["Id_dem","Id_dep","Id_arr","User"])
% demands = createDemand(nav,1,2,2.23936861455827,48.6251716749029,3.344755124419394e-05/2,2.239323171837208,48.625162383652835,3.344755628968206e-05/2,demands);






[pathOutput,solutionInfo]=GetPath(nav,1,1);


pth = Path(pathOutput,solutionInfo,nav,0);
show_path(nav,pathOutput,solutionInfo);
