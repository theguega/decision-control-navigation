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
 
function Donnees = VariablesCommandeAttraction(Data)
%Donnees = Donnees = [Ecart; ThetaTilde; Ex; Ey; Theta]; %Ecart Robot-Cible,
global Xd Yd
 
%Etat actuel du Robot Mobile
Xreel = Data(1);
Yreel = Data(2);
Theta = Data(3);
 
Ex = Xd - Xreel; % Détermination des erreurs de position en X
Ey = Yd - Yreel; % Détermination des erreurs de position en Y
Ecart = sqrt((Ex^2)+(Ey^2)); % Détermination de l'écart entre le robot mobile et le point désiré
 
%Détermination de l'angle thetaC (Theta Consigne)
ThetaC = atan2(Ey,Ex); 
%%
%Détermination de l'erreur angulaire (ThetaTilde = ThetaC - Theta)
ThetaTilde=SoustractionAnglesAtan2(ThetaC, Theta);
 
%Paramètres de Commande
Donnees = [Ecart; ThetaTilde; Ex; Ey; Theta];


