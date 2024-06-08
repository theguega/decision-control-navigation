function res = EquationDiff_Tourbillon(t, Vecteur, RayonCycleLimite)
    %XP = systBO(t,X)
    % Cette fonction retourne le vecteur Xpoint Ypoint du systeme

    X  = Vecteur(1);
    Y  = Vecteur(2);
    
    mu=0.05;
    Xp = Y + X*mu*(RayonCycleLimite^2 - X^2 - Y^2);
    Yp = -X + Y*mu*(RayonCycleLimite^2 - X^2 - Y^2);
    res = [Xp; Yp];
end