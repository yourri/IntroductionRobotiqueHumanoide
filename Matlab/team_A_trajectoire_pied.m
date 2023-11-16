
function [trajectoire_pied_x] =  creation_trajectoire_pied_x(Xi, Xf, Ti, Tf)
    Vix=0;
    Vfx=0;


    ax=Vix;
    cx=(-2*(Xf-Xi))*Tf^3;
    bx=(-3*cx/2)*Tf;
    dx=Xi;

    t=[Ti:Ts:Tf]';

    trajectoire_pied_x = cx*t.^3 + bx*t.*t + ax*t + dx;
end

function [trajectoire_pied_y] =  creation_trajectoire_pied_y(Yi, Yf, Ti, Tf)
    Viy=0;
    Vfy=0;

    ay=Viy;
    cy=(-2*(Yf-Yi))*Tf^3;
    by=(-3*cy/2)*Tf;
    dy=Yi;


    trajectoire_pied_y = cy*t.^3 + by*t.*t + ay*t + dy;
end

function [trajectoire_pied_Z] = creation_trajectoire_pied_z(Zi, Zf, Ti, Tf)
    deltaZ=Zf-Zi;
    zmax  =0.02;

    Viz   =0;
    Vfz   =0; %mètres par seconde

    syms a b c d e
    eqns = [a == Zi, b == Viz, a+b*Tf+c*Tf^2+d*Tf^3+e*Tf^4 == Zf, a+b*Tf/2+c*(Tf/2)^2+d*(Tf/2)^3+e*(Tf/2)^4 == zmax, b+2*c*Tf+3*d*Tf^2+4*e*Tf^3 == 0]; 
    [az, bz, cz, dz, ez] = solve(eqns, [a b c d e]);

    trajectoire_pied_Z = ez*t.^4 + dz*t.^3 + cz*t.*t + bz*t + az;
end