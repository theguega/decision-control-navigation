clear, clc, close all;

%pos_obst = load("obstacles.txt");
pos_obst=[];
targets = load("target.txt");

%init obstacles
obstacles = [];
for i = 1:size(pos_obst, 1)
    obstacles = [obstacles, obstacle(pos_obst(i, 1), pos_obst(i, 2), pos_obst(i, 3), pos_obst(i, 4))];
    obstacles(i).plot();
end

%init vehicle
robot = vehicle(targets(1,1), targets(1,2), 0, 0, 0);
robot.plot()

hold on, grid on;

% Plot all targets
for i = 1:size(targets, 1)
    plot(targets(i, 1), targets(i, 2), 'k+', 'LineWidth', 2);
end

% Go through each target
for j = 1:size(targets, 1)
    target = targets(j, :);
    plot([robot.getX(); target(1)], [robot.getY(); target(2)], ':');
    
    % Move towards the target until reaching it
    i=0
    while(robot.get_distance_point(target) > 1)
        res = robot.controller_selection(obstacles, target);
        robot.update_pos(res);
        i=i+1
        if (~rem(i, 10)) %tous les multiple de 10
            robot.plot();
        end
    end
end