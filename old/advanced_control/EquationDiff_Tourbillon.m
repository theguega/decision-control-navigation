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

function XP = EquationDiff_Tourbillon(t, Vecteur)
%XP = systBO(t,X)
% Cette fonction retourne le vecteur Xpoint du systeme

global RayonCycleLimite Mu

X  = Vecteur(1);
Y  = Vecteur(2);

Xp = Y + X*(RayonCycleLimite^2 - X^2 - Y^2);
Yp = -X + Y*(RayonCycleLimite^2 - X^2 - Y^2);

XP = [Xp; Yp];

end
