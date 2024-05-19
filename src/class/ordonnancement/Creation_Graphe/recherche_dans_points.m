function pos = recherche_dans_points(x, y, points)
    pos = NaN;
    for i = 1:size(points, 1)
        disp(i);
        if points(i,1) == x && points(i, 2) == y
            pos = i;
            return;
        end
    end
end