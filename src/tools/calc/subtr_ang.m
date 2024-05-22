function E_Theta=subtr_ang(Theta_T, ThetaReel)
    E_Theta = Theta_T - ThetaReel;
    if E_Theta > pi
        E_Theta =  E_Theta - 2*pi;
    elseif E_Theta < -pi
        E_Theta = E_Theta + 2*pi;
    end
    if abs(E_Theta) > (pi/2)
        E_Theta = sign(E_Theta)*((pi/2) - 0.06);
    end
end