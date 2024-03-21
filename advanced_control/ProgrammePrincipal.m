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
%% Etudiant :
%% Date :
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
close all  
clc, clear all,
 
global Xreel0 Yreel0 Theta0                             %Conditions initiales du robot
global Rrobot                                           %Rayon du robot
global XLast YLast                                      %Utile uniquement pour l'aspect affichage
global L d r                                            %Paramètres géométriques du robot
global AccelerationLineaireMax AccelerationAngulaireMax %Caractéristiques physiques des actionneurs
 
global TempsCourant inc2
global S_DistanceRobotCible S_AngleRobotCible S_DistanceRobotObstacle1 S_DistanceVerticleRobotObstacle1 S_IndiceObstaclePlusProche  S_SensEvitement %Pour la sauvegarde des cracatéristiques de la simulation
 
global CommandeReelle_OLD                               %Sauvegarde de la commande précédente
global IndicateurCA                                     %Indicateur Comportement Actif
global IndicateurCA_OLD                                 %Pour la sauvegarde IndicateurCA précédent
global SensEvitementObstacle_Old     %Pour sauvegarder le sens de l'évitement d'obstacle pour l'itération d'après (-1 -> counter-clockwise, 1 -> clockwise and 0 don't avoiding before or else)
global IndiceObstaclePlusProche_Old  %Pour sauvegarder l'indice de l'obstacle sur lequel s'applique l'evitement.
global G_ActivationAlgorithmeOrbital %Pour choisir l'activation de l'algo orbital (==1) ou pas (==0)
 
global Duree                         %Durée de la simulation souhaitée
global Xd Yd                         %Coordonnées du point désirée à atteindre
global Obstacle                      %Déclaration de la liste d'obstacles
global RayonCycleLimite              %Rayon du cycle limite
global G_Affichage                   %Variable pour activer ou pas l'affichage graphique des résultats
 
%%Paramètres de la simulation
Duree = 40;                          %Temps de la simulation en secondes
PasEchantillonnage = 0.1;            %Pas d'échantillonnage constant pour la simulation sous Simulink
G_Affichage = 1;                     %Affichage ou pas des simulations
G_ActivationAlgorithmeOrbital = 0;   %Activation ou pas de l'algorithme orbital
%%
%%Conditions initiales du robot
Xreel0 = 5;
Yreel0 = 20;
Theta0 = deg2rad(45);
Rrobot = 0.8;                        %Choix pour le rayon du cercle qui entoure le robot
%%
%%Coorodonnées dans l'environnement du point à atteindre
Xd = 20; Yd = 12;    %Position de la cible (phase de test de l'attraction vers la cible"

%%
ModeleRobotUnicycle();    %Fonction qui permet de donner le modèle (géométrique, cinématique, caractéristiques des actionneurs, etc.) du robot unicyle utilisé
%%
%Variables pour afficher le temps de la simulation (dans le fichier CoordinationControleurs)
SemaphoreTempsImpactCible = 1;
inc2 = 0;
%%
SensEvitementObstacle_Old = 0;    %Pas d'évitement initialement
IndiceObstaclePlusProche_Old = 0;
IndicateurCA_OLD = -1;            %On lui affecte initialement "-1" pour dire qu'on commence la simulation (donc pas de contrôleur actif)
W_TrajectoireReelle = [];
 
if G_Affichage ~= 0
    close all
    figure(1);
    hold on
end

PositionsObstacles = load("obstacles.txt");
 
%%Instanciation des obstacles utilisés dans la simulation
Obstacle = CObstacle(); %Initialisation de la structure
for i=1:size(PositionsObstacles, 1)
    Obstacle(i) = CObstacle(PositionsObstacles(i,:), 0.8, 0.5, [0, 0]);%%Position, Rayon, Marge, [VitesseLinéaire VitesseAngulaire]
end
 
%%Affichage des différents éléments de la simulation
if G_Affichage ~= 0
    DessinUnicyle(Xreel0,Yreel0,Theta0,'');
    for i=1:size(Obstacle, 2)
        Affichage(Obstacle(i), 1);
        text(GetX(Obstacle(i)), GetY(Obstacle(i))+0.5, num2str(i)); %%Pour le dessin de l'indice de l'obstacle
    end
 
    %%Tracé de la trajectoire consigne du robot mobile (en ne considérant pas les obstacles)
    hold on,grid on,
    axis([0 25 15 25]),
    plot(Xreel0,Yreel0,'k+','LineWidth',2)
    plot(Xd,Yd,'k+','LineWidth',2)
    plot([Xreel0;Xd],[Yreel0;Yd],':')
    title('Trajectoire du robot dans le repère [O, X, Y]')
    xlabel('X [m]')
    ylabel('Y [m]')
end

%%Etat précédent du robot mobile juste avant de lancer la simulation sous Simulink
XLast = Xreel0;
YLast = Yreel0;
[VecteurTemps,E,O] = sim('ArchitectureControleNavigation', [0 Duree], simset('Solver','ode1','FixedStep', PasEchantillonnage));
 
if G_Affichage ~= 0
    %% Trajectoire réelle du robot mobile dans son environnement
    hold on
    plot(W_TrajectoireReelle(:,1),W_TrajectoireReelle(:,2),'-m');
end
%%%%%%%%%%%%%%%%%%%%%%%%
%Affichage des résultats%
%%%%%%%%%%%%%%%%%%%%%%%%
if G_Affichage ~= 0
    figure,
    plot(VecteurTemps,W_ValeurFonctionLyapunov,'r')
    hold on
    plot(VecteurTemps,200*W_IndicateurSauvegarde)
    title('Evolution de la fonction de Lyapunov des deux contrôleurs')
    xlabel('[s]');
    grid on;
    %%maximize('all'); %Fonction pour maximiser la taille de toutes les figures
end


