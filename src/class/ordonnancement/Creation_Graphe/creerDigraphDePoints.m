function monDigraph = creerDigraphDePoints(points,edges)
    % points est un matrice N*2 (X,Y,id) et edges une matrice M*3
    % (Predécesseur, Successeur, Poid)
    monDigraph = digraph();
    
    for i = 1:size(points, 1)
        monDigraph = addnode(monDigraph,sprintf('Noeud %d.', i));
    end
    
    % Ajouter les arêtes au graphe
    for j = 1:size(edges, 1)
        monDigraph = addedge(monDigraph,edges(j,1),edges(j,2),edges(j,3));
    end
end