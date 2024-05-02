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

function CommandeReelle = CommandeEvitement(Donnees)
%%Donnees = [ThetaTilde; ThetaC_p];

ThetaTilde = Donnees(1);
ThetaC_p = Donnees(2);

%Vmax = 1; %1m/s -> 3.6km/h
Vmax = 0.5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Premier type de commande
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Kp = 15;
V = Vmax;
W = ThetaC_p + Kp*ThetaTilde;

ValeurFonctionLyapunov = 0.5*rad2deg(ThetaTilde)^2/10;

CommandeReelle = [V, W, ValeurFonctionLyapunov];
