 clear, clc, close all;

%INIT MAPS - OBSTACLES - MAP - VEHICULE
scenario = drivingScenario;
roadNetwork(scenario,'OpenStreetMap','utac.osm');
plot(scenario);
hold on;
xlim([-50 150]);
ylim([-30 80]);

pos_obst = load("obstacles.txt");
num_obstacles = size(pos_obst, 1); 
obstacles = repmat(obstacle(0,0,0,0), 1, num_obstacles);
for i = 1:num_obstacles
    obstacles(i) = obstacle(pos_obst(i, 1), pos_obst(i, 2), pos_obst(i, 3), pos_obst(i, 4));
    obstacles(i).plot();
end

targets = load("target.txt");
for i = 1:size(targets, 1)
    plot(targets(i, 1), targets(i, 2), 'k+', 'LineWidth', 2);
end

agent = vehicle(targets(1,1), targets(1,2), 0, 0, 0, obstacles, targets);

%update vehicle
i=0;

while true

    dt=0.1;
    agent.update(dt);

    %just for plot
    i=i+1;
    if (~rem(i, 30)) %tous les multiple de 10
        agent.plot();
        drawnow;
    end
end