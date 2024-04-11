clear, clc, close all;

targets = [20, 20];
pos_obst = [8,10,1,2];


obstacles = [];
figure;
hold on;

for i = 1:size(pos_obst, 1)
    obstacles = [obstacles, obstacle(pos_obst(i, 1), pos_obst(i, 2), pos_obst(i, 3), pos_obst(i, 4))];
    obstacles(i).plot();
    drawnow;
end

%init vehicle
robot = vehicle(0, 0, 0, 0, 0);
robot.plot()


% Plot all targets
for i = 1:size(targets, 1)
    plot(targets(i, 1), targets(i, 2), 'k+', 'LineWidth', 2);
end

% Go through each target
for j = 1:size(targets, 1)
    target = targets(j, :);
    plot([robot.getX(); target(1)], [robot.getY(); target(2)], ':');
    
    % Move towards the target until reaching it
    i=0;
    while(robot.get_distance_point(target) > 1)
        res = robot.controller_selection(obstacles, target);
        robot.update_pos(res);
        i=i+1;
        if (~rem(i, 50)) %tous les multiple de 10
            robot.plot();
            drawnow;
        end
    end
end