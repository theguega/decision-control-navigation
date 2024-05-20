clear, clc, close all;
hold on;

%---------------------------- OBSTACLES - MAP - VEHICULE ----------------------------
pos_obst = load("data/obstacles.txt"); %obstacles = [x, y, marge]
num_obstacles = size(pos_obst, 1);
obstacles = repmat(obstacle(0,0,0,0), 1, num_obstacles);
for i = 1:num_obstacles
    obstacles(i) = obstacle(pos_obst(i, 1), pos_obst(i, 2), pos_obst(i, 3), pos_obst(i, 4));
    obstacles(i).plot();
end

% targets for agent 1
pos_targets1 = load("data/targets1.txt"); % targets = [x, y]
num_targets1 = size(pos_targets1, 1);
targets1 = repmat(target(0,0,0,0,0), 1, num_targets1);
for i = 1:num_targets1
    if i == num_targets1
        theta_target= 0;
    else
        theta_target = atan2(pos_targets1(i+1,2)-pos_targets1(i,2), pos_targets1(i+1,1)-pos_targets1(i,1));
    end
    targets1(i) = target(pos_targets1(i, 1), pos_targets1(i, 2), theta_target, 0, 0);
    targets1(i).plot();
end

% targets for agent 2
pos_targets2 = load("data/targets2.txt"); % targets = [x, y]
num_targets2 = size(pos_targets2, 1);
targets2 = repmat(target(0,0,0,0,0), 1, num_targets2);
for i = 1:num_targets2
    if i == num_targets2
        theta_target= 0;
    else
        theta_target = atan2(pos_targets2(i+1,2)-pos_targets2(i,2), pos_targets2(i+1,1)-pos_targets2(i,1));
    end
    targets2(i) = target(pos_targets2(i, 1), pos_targets2(i, 2), theta_target, 0, 0);
    targets2(i).plot();
end

agent = vehicle(targets1(1).x, targets1(1).y, targets1(1).theta, 0, obstacles, targets1, [],1, 0, 0);
agent2 = vehicle(targets2(1).x, targets2(1).y, targets2(1).theta, 0, obstacles, targets2, [],2, 0 ,0); % vehicle
agent.update_acc_vehicles(agent2)
agent2.update_acc_vehicles(agent)

%---------------------------- UPDATE VEHICLE POS ----------------------------
i=0;
dt=0.05; % fixed sample time
while true
    if (isempty(agent.targets) && isempty(agent2.targets))
        break;
    end

    if (~isempty(agent.targets))
        agent.update(dt);
        if (~rem(i, 10))
            agent.plot2();
            drawnow;
        end
    end

    if (~isempty(agent2.targets))
        agent2.update(dt);
        if (~rem(i, 10))
            agent2.plot2();
            drawnow;
        end
    end
    
    i=i+1;
end

%agent.plot_corrector_action