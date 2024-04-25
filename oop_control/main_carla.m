% To run this file, you need to have the assets/ and src/ folders from the
% simulation repo in your MATLAB path.

clear, clc, close all;

%INIT MAPS - OBSTACLES - MAP - VEHICULE
scenario = drivingScenario;
roadNetwork(scenario,'OpenDRIVE','scene.xodr');
plot(scenario);
hold on;
xlim([-150 150]);
ylim([-150 150]);

pos_obst = load("obstacles.txt");
num_obstacles = size(pos_obst, 1); 
obstacles = repmat(obstacle(0,0,0,0), 1, num_obstacles);
for i = 1:num_obstacles
    obstacles(i) = obstacle(pos_obst(i, 1), pos_obst(i, 2), pos_obst(i, 3), pos_obst(i, 4));
    obstacles(i).plot();
end

simulator = Simulator();

targetRoads = [63 124 42 164];
agent = VehicleCarla(simulator, -75.9120559692383, -46.4463195800781, 0, 0, 0, obstacles);
for roadId = targetRoads
    agent.addTargetRoad(roadId);
end
    
for i = 1:size(agent.targets, 1)
    plot(agent.targets(i, 1), agent.targets(i, 2), 'k+', 'LineWidth', 2);
end


% i=0;

dt = 0;
while true
    tic();
    agent.update(dt);

    % just for plot
    % i=i+1;
    % if (~rem(i, 30)) %tous les multiple de 10
    %     agent.plot();
    %     drawnow;
    % end

    dt = toc() * 5;
end