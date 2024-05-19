clear, clc, close all;

%---------------------------- INIT MAPS - OBSTACLES - MAP - VEHICULE ----------------------------
%scenario = drivingScenario; %map
%roadNetwork(scenario,'OpenStreetMap','data/utac.osm');
%plot(scenario);
hold on;
xlim([-10, 210])
ylim([-20, 20])

pos_obst = load("data/obstacles.txt"); %obstacles = [x, y, marge]
num_obstacles = size(pos_obst, 1);
obstacles = repmat(obstacle(0,0,0,0), 1, num_obstacles);
for i = 1:num_obstacles
    obstacles(i) = obstacle(pos_obst(i, 1), pos_obst(i, 2), pos_obst(i, 3), pos_obst(i, 4));
    obstacles(i).plot();
end

% pos_targets = load("data/control.txt"); % targets = [x, y]
% num_targets = size(pos_targets, 1);
% targets = repmat(target(0,0,0,0,0), 1, num_targets);
% for i = 1:num_targets
%     if i == num_targets
%         theta_target= 0;
%     else
%         theta_target = atan2(pos_targets(i+1,2)-pos_targets(i,2), pos_targets(i+1,1)-pos_targets(i,1));
%     end
%     targets(i) = target(pos_targets(i, 1), pos_targets(i, 2), theta_target, 0, 0);
%     targets(i).plot();
% end

%test for moving target
targets1 = [target(15, 0, 0, 0, 0), target(200, 0, 0, 0, 0.001)];
targets2 = [target(0, 0, 0, 0, 0), target(200, 0, 0, 0, 0)];
targets1(1).plot()
targets1(2).plot()

agent = vehicle(targets1(1).x, targets1(1).y, 0, 0, obstacles, targets1, [],1);
agent2 = vehicle(targets2(1).x, targets2(1).y, 0, 0, obstacles, targets2, agent,2); % vehicle

%---------------------------- UPDATE VEHICLE POS ----------------------------
i=0;
dt=0.05; % fixed sample time
while true
    if (isempty(agent.targets) || isempty(agent2.targets))
        break;
    end
    
    agent.update(dt);
    agent2.update(dt);
    
    i=i+1;
    if (~rem(i, 10))
        agent.plot();
        agent2.plot();
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
