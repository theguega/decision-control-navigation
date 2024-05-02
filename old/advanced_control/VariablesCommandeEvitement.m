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

function Donnees = VariablesCommandeEvitement(Data, IndiceObstaclePlusProche, DistanceRobotObstaclePlusProche, SensEvitementObstacle)
%Donnees = [Ecart; ThetaTilde; ThetaC_p]; %Ecart Robot-Cible,
%ThetaTilde=Theta_C-Theta; "SensEvitementObstacle" paramètre pour forcer ou pas le sens de l'evitement d'obstacle 

%%%%%%%%%%%%
global Xd Yd
global Obstacle
global RayonCycleLimite Mu
global IndiceObstaclePlusProche_Old

%Etat actuel du Robot Mobile
Xreel = Data(1);
Yreel = Data(2);
Theta = Data(3);

%%Détermination de la position relative du robot par rapport à l'obstacle (car le cycle limite est donné par rapport
%%au centre de l'obstacle et non au centre du repère (0,0)
PositionObstacle = GetPosition(Obstacle(IndiceObstaclePlusProche));
Xrelatif = Xreel - PositionObstacle(1); %X robot-obstacle
Yrelatif = Yreel - PositionObstacle(2); %Y robot-obstacle

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Méthode pour le changenement de repères
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Y_D_O = Yd - PositionObstacle(2); %Différence D : Point désirée (target) et Yobst
X_D_O = Xd - PositionObstacle(1);

Alpha = atan2(Y_D_O, X_D_O);

%Calcul de la matrice de passage du repère obtstacle (R_O) au repère absolu (R_A)
T_O_A = inv ([cos(Alpha) -sin(Alpha) 0 PositionObstacle(1)
              sin(Alpha) cos(Alpha) 0 PositionObstacle(2)
              0 0 1 0
              0 0 0 1]);
          
CoordonneeRepereObstacle = T_O_A * [Xreel; Yreel; 0; 1];

X_Prime = CoordonneeRepereObstacle(1); %Attention le "prime" ici ne veut pas dire dérivée
Y_Prime = CoordonneeRepereObstacle(2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Obtention du Rayon du cycle-limite à suivre
if (X_Prime <= 0) %Alors Attraction vers le cycle limite 
    RayonCycleLimite=GetRv(Obstacle(IndiceObstaclePlusProche))-0.3;
else %Période de l'extraction du cycle limite 
    RayonCycleLimite=RayonCycleLimite+0.03; %Pour que le changement de rayon soit en douceur. 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Partie pour forcer ou pas le sens de rotation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if SensEvitementObstacle ~= 0 %=> Qu'il faut forcer le sens de rotation
    Y_Prime = SensEvitementObstacle;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Calcul de l'orientation "ThetaC" à faire suivre par le robot pour suivre le cycle limite 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
disp(Xrelatif)
disp(Yrelatif)
disp(RayonCycleLimite)
disp("----")
X_dot = -Yrelatif + Xrelatif * ((RayonCycleLimite^2) - (Xrelatif^2) - (Yrelatif^2));
Y_dot = -Xrelatif + Yrelatif * ((RayonCycleLimite^2) - (Xrelatif^2) - (Yrelatif^2));
ThetaC = atan2(Y_dot, X_dot);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Procédure pour récupérer ThetaC_p
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
X0 = [X_dot, Y_dot];
[t,Xc] = ode23('EquationDiff_Tourbillon',[0, 0.2],X0);
ThetaC_p = atan2((Xc(2,2)-Xc(1,2)), (Xc(2,1)-Xc(1,1)));

%%%%
ThetaTilde=SoustractionAnglesAtan2(ThetaC, Theta);

%Paramètres de commande
Donnees = [ThetaTilde; ThetaC_p; sign(Y_Prime)];