% To run this file, you need to have the assets/ and src/ folders from the
% simulation repo in your MATLAB path.

clear, clc, close all;

%---------------------------- INIT MAPS - OBSTACLES - MAP - VEHICULE ----------------------------
scenario = drivingScenario; %map
roadNetwork(scenario,'OpenDRIVE','scene.xodr');
plot(scenario);
hold on;
xlim([-150 150]);
ylim([-150 150]);

pos_obst = load("data/obstacles.txt"); %obstacles
num_obstacles = size(pos_obst, 1);
obstacles = repmat(obstacle(0,0,0,0), 1, num_obstacles);
for i = 1:num_obstacles
    obstacles(i) = obstacle(pos_obst(i, 1), pos_obst(i, 2), pos_obst(i, 3), pos_obst(i, 4));
end

simulator = Simulator();

targetRoads = [63 124 42 164];
agent = VehicleCarla(simulator, -75.9120559692383, -46.4463195800781, 0, obstacles);
for roadId = targetRoads
    agent.addTargetRoad(roadId);
end


%---------------------------- UPDATE VEHICLE POS ----------------------------
dt=0.05; % fixed sample time
while true
    if isempty(agent.targets)
        break;
    end

    agent.update(dt);
end