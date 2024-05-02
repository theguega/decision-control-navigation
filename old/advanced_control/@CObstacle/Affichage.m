%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lounis ADOUANE                                                     %
%% Université de Technologie de Compiègne (UTC)                       %
%% SY28 :: Département Génie Informatique (GI)                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Part I : Planning and control of mobile robots                     %
%% Théorème de stabilité de Lyapunov et méthode des cycles-limites    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Dernière modification le 21/03/2022                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Affichage(Obstacle, iFigure)%(Xobst,Yobst,Robstacle, Rv, iFigure)

global Rrobot

%Détermination de Xobstacle = f(Yobstacle) (contour de l'obstacle réel)
gamma = [0:0.01:2*pi]';
Xobstacle = Obstacle.Rayon * cos(gamma) + Obstacle.Position(1);
Yobstacle = Obstacle.Rayon * sin(gamma) + Obstacle.Position(2);

%=============================================
% Région de détection autour de l'obstacle  ||
%=============================================
Xdetection = GetRv(Obstacle) * cos(gamma) + Obstacle.Position(1);
Ydetection = GetRv(Obstacle) * sin(gamma) + Obstacle.Position(2);

%Tracé de l'obstacle
figure(iFigure)
hold on
plot(Obstacle.Position(1),Obstacle.Position(2),'k+','LineWidth',1)
plot(Xobstacle,Yobstacle,'k','LineWidth',2)
plot(Xdetection,Ydetection,'k:','LineWidth',2)
axis equal
grid on