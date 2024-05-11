clear, clc, close all;

%---------------------------- INIT MAPS - OBSTACLES - MAP - VEHICULE ----------------------------
%scenario = drivingScenario; %map
%roadNetwork(scenario,'OpenStreetMap','data/utac.osm');
%plot(scenario);
hold on;

pos_obst = load("data/obstacles.txt"); %obstacles = [x, y, marge]
num_obstacles = size(pos_obst, 1);
obstacles = repmat(obstacle(0,0,0,0), 1, num_obstacles);
for i = 1:num_obstacles
    obstacles(i) = obstacle(pos_obst(i, 1), pos_obst(i, 2), pos_obst(i, 3), pos_obst(i, 4));
    obstacles(i).plot();
end

pos_targets = load("data/control.txt"); % targets = [x, y]
num_targets = size(pos_targets, 1);
targets = repmat(target(0,0,0,0,0), 1, num_targets);
for i = 1:num_targets
    if i == num_targets
        theta_target= 0;
    else
        theta_target = atan2(pos_targets(i+1,2)-pos_targets(i,2), pos_targets(i+1,1)-pos_targets(i,1));
    end
    targets(i) = target(pos_targets(i, 1), pos_targets(i, 2), theta_target, 0, 0);
    targets(i).plot();
end

agent = vehicle(targets(1).x, targets(1).y, 0, 0, obstacles, targets); % vehicle

%---------------------------- UPDATE VEHICLE POS ----------------------------
i=0;
dt=0.05; % fixed sample time
while true
    if isempty(agent.targets)
        break;
    end
    
    agent.update(dt);
    targets(2).update(dt)
    targets(2).plot
    
    i=i+1;
    if (~rem(i, 10)) %every 30 iterations
        agent.plot();
        drawnow;
    end
end

error1=agent.theta_error_output;
figure(2);
plot(error1(:,1),'red')
hold on;
plot(error1(:,2),'blue')

error2=agent.speed_output;
figure(3);
plot(error2(:,1),'red')
