%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lounis ADOUANE                                                     %
%% Universit� de Technologie de Compi�gne (UTC)                       %
%% SY28 :: D�partement G�nie Informatique (GI)                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Part I : Planning and control of mobile robots                     %
%% Th�or�me de stabilit� de Lyapunov et m�thode des cycles-limites    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Derni�re modification le 21/03/2022                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Obstacle = CObstacle(Position, Rayon, Marge, Vitesse)
% CObstacle class constructor
% Cr�er un Obstacle avec les param�tres d'entr�e
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
