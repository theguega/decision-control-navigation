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

function Obstacle = CObstacle(Position, Rayon, Marge, Vitesse)
% CObstacle class constructor
% Créer un Obstacle avec les paramètres d'entrée
global Rrobot

if nargin == 0
    Obstacle.Position = [0 0];
    Obstacle.Vitesse = [0 0];
    Obstacle.Rayon = 0;
    Obstacle.Marge = 0;
    Obstacle.Rv = 0;
    Obstacle = class(Obstacle,'CObstacle');
else
    Obstacle.Position = Position;
    Obstacle.Vitesse = Vitesse;
    Obstacle.Rayon = Rayon;
    Obstacle.Marge = Marge;
    Obstacle.Rv = Rrobot + Rayon + Marge;
    Obstacle = class(Obstacle, 'CObstacle');
end
