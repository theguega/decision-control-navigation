clear;
clc;

coordinates = load('points.txt');
window_size = 20;
figure;
ax = gca;
axis(ax, [-10 10 -10 10]);

vehicle = AutonomousVehicle(0, 0, 0, 0);

plot(ax, coordinates(:, 1), coordinates(:, 2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
hold on;

cruising_speed = 2;
arrival_speed = 0.75;

for i = 1:size(coordinates, 1)
    target_x = coordinates(i, 1);
    target_y = coordinates(i, 2);
    
    % Use moveToPoint method
    disp(['Moving to point (' num2str(target_x) ', ' num2str(target_y) ')']);
    disp(['Vehicle pos : (' num2str(vehicle.x) ', ' num2str(vehicle.y) ')']);
    vehicle.moveToPoint(target_x, target_y, cruising_speed, arrival_speed, ax);
end