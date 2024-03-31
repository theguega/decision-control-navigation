clear, clc, close all;

pos_obst=load("obstacles.txt");
target=load("target.txt");

%init obstacles
obst = [];
for i=1:size(pos_obst,1)
    obst=[obst, obstacle(pos_obst(i,1),pos_obst(i,2),pos_obst(i,3),pos_obst(i,4))];
    obst(i).plot();
end

%init vehicle
robot = vehicle(0, 0, 0, 0, 0);
robot.plot()

%go to target
hold on,grid on,
plot(robot.getX(),robot.getY(),'k+','LineWidth',2)
plot(target(1),target(2),'k+','LineWidth',2)
plot([robot.getX();target(1)],[robot.getY();target(2)],':')
title('Trajectoire du robot dans le repere [O, X, Y]')
xlabel('X [m]')
ylabel('Y [m]')

