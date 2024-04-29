function res = EquationDiff_Tourbillon(t, Vecteur, RayonCycleLimite)
    %XP = systBO(t,X)
    % Cette fonction retourne le vecteur Xpoint Ypoint du systeme

    X  = Vecteur(1);
    Y  = Vecteur(2);

    Xp = Y + X*(RayonCycleLimite^2 - X^2 - Y^2);
    Yp = -X + Y*(RayonCycleLimite^2 - X^2 - Y^2);
    res = [Xp; Yp];
end