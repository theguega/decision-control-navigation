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
 
function Affichage(Data)
 
global XLast YLast
global Delta
global TempsCourant
global IndicateurCA
global G_Affichage
global Simulation_type  
global Xd Yd
 
if G_Affichage ~= 0
    Xreel = Data(1);
    Yreel = Data(2);
    Theta = Data(3);
 
    %%% Affichage Graphique
    figure(1)
    hold on
    
    %%%Tracé de l'état actuel du robot mobile chaque intervalle déterminé
    if ~((Xreel==0) & (Yreel==0) & (Theta==0))
        if sqrt((Xreel-XLast)^2 + (Yreel-YLast)^2) > 0.7
            %DessinUnicyle(Xreel,Yreel,Theta, TempsCourant);
            DessinUnicyle(Xreel,Yreel,Theta, '');
            %DessinEnveloppeCycab(Xreel,Yreel,Theta);
            XLast = Xreel;
            YLast = Yreel;
            %%Tracé de la distance entre le Robot Mobile et l'obstacle
            %plot([Xreel;Xobst],[Yreel;Yobst],'m:','LineWidth',2)
        end
    end
end


