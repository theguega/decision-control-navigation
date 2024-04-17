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

function ModeleRobotUnicycle

global L d r 
global AccelerationLineaireMax AccelerationAngulaireMax
global Rrobot

% Paramètres géométriques du robot mobile
L = 2.5;     % Empattement (en mètre)
d = Rrobot;  % Largeur des essieux (en mètre) pour que ça colle à la définition du robot circulaire
r = 1;     % Rayon des roues (en mètre)

AccelerationLineaireMax = 1;
AccelerationAngulaireMax = 1;
