% Numerical inverse kinematics with random target
% ik_random2.m
% 2004 Dec.17 s.kajita AIST

close all
clear              % clear work space
global uLINK   
global temps_un_pas 
global Ts
global comx
global comy
global dX

Ts = 0.05;
Temps_initial = 2;
teamA_setup_darwin; 
team_A_ZMP_trajectory;
% Set the biped robot of Fig.2.19 and Fig.2.20

athigh = atan(0.04 / 0.093);
atibia = atan(0.04 / 0.093);

team_A_trajectoire_pied;

%%%%%%%%%%% set non singular posture@%%%%%%%%%%%%
uLINK(MP_THIGH1_L).q = 0.3;
uLINK(MP_THIGH2_L).q = athigh;
uLINK(MP_TIBIA_L).q = -(athigh+atibia);
uLINK(MP_ANKLE1_L).q = -(atibia);
uLINK(MP_ANKLE2_L).q = (0.3);

uLINK(MP_THIGH1_R).q = 0.3;
uLINK(MP_THIGH2_R).q = -athigh;
uLINK(MP_TIBIA_R).q = (athigh+atibia);
uLINK(MP_ANKLE1_R).q = (atibia);
uLINK(MP_ANKLE2_R).q = (0.3);

uLINK(MP_BODY).p = [0.0, 0.0, 0.30]';
uLINK(MP_BODY).R = eye(3);

%%%%%%%%%%%random target foot position and orientation %%%%%%%%%%%%

ForwardKinematics(1);

p_ankle_r_int = uLINK(MP_ANKLE2_R).p;
p_ankle_l_int = uLINK(MP_ANKLE2_L).p;
p_body = uLINK(MP_BODY).p;

 
for i=1:7
    if mod(i, 2) == 1 
        % Trajectoire pour le pied gauche
        trajectoire_x = creation_trajectoire_pied_x(p_ankle_l_int(1), p_ankle_l_int(1) + dX, 0, temps_un_pas, Ts);
        trajectoire_y = creation_trajectoire_pied_y(p_ankle_l_int(2), p_ankle_l_int(2), 0, temps_un_pas, Ts);
        trajectoire_z = creation_trajectoire_pied_z(p_ankle_l_int(3), p_ankle_l_int(3), 0, temps_un_pas, Ts);
    else
        % Trajectoire pour le pied droit
        trajectoire_x = creation_trajectoire_pied_x(p_ankle_r_int(1), p_ankle_r_int(1) + dX*2, 0, temps_un_pas, Ts);
        trajectoire_y = creation_trajectoire_pied_y(p_ankle_r_int(2), p_ankle_r_int(2), 0, temps_un_pas, Ts);
        trajectoire_z = creation_trajectoire_pied_z(p_ankle_r_int(3), p_ankle_r_int(3), 0, temps_un_pas, Ts);
    end

    for j=1+((i-1) * length(trajectoire_x)):1+((i-1) * length(trajectoire_x)) + length(trajectoire_x)
        index = j - (i-1) * length(trajectoire_x);
        if mod(i,2) == 1
            Lfoot.p = p_ankle_l_int + [trajectoire_x(index) trajectoire_y(index) trajectoire_z(index)']';
            Lfoot.R = RPY2R(1/2*pi*[0, 0, 0]);  %  -pi/4 < q < pi/4
            rerr_norm1 = InverseKinematics(MP_ANKLE2_L, Lfoot);     
        else
            Rfoot.p = p_ankle_r_int + [trajectoire_x(index) trajectoire_y(index) trajectoire_z(index)']';
            Rfoot.R = RPY2R(1/2*pi*[0, 0, 0]); 
            rerr_norm1 = InverseKinematics(MP_ANKLE2_R, Rfoot);
        end
        uLINK(MP_BODY).p = p_body + [comx(j)+0.015 comy(j) 0]';
        
        clf
        DrawAllJoints(1);
        view(38,10)
        axis equal
        zlim([-0.2 1.2])
        grid on

        fprintf('Type any key for another pose, Ctrl-C to abort\n');
        pause
    end   
end


function [trajectoire_pied_x] =  creation_trajectoire_pied_x(Xi, Xf, Ti, Tf, Ts)
    Vix=0;
    Vfx=0;


    ax=Vix;
    cx=(-2*(Xf-Xi))/Tf^3;
    bx=(-3*cx/2)*Tf;
    dx=Xi;

    t=[Ti:Ts:Tf]';

    trajectoire_pied_x = cx*t.^3 + bx*t.*t + ax*t + dx;
end

function [trajectoire_pied_y] =  creation_trajectoire_pied_y(Yi, Yf, Ti, Tf, Ts)
    Viy=0;
    Vfy=0;

    ay=Viy;
    cy=(-2*(Yf-Yi))/Tf^3;
    by=(-3*cy/2)*Tf;
    dy=Yi;

    t=[Ti:Ts:Tf]';
    trajectoire_pied_y = cy*t.^3 + by*t.*t + ay*t + dy;
end

function trajectoire_pied_Z = creation_trajectoire_pied_z(Zi, Zf, Ti, Tf, Ts)
    deltaZ = Zf - Zi;
    zmax = 0.02;

    Viz = 0;
    Vfz = 0; % mètres par seconde

    t = Ti:Ts:Tf;

    A = [1, Ti, Ti^2, Ti^3, Ti^4; 
         0, 1, 2*Ti, 3*Ti^2, 4*Ti^3; 
         1, Tf, Tf^2, Tf^3, Tf^4; 
         0, 1, 2*Tf, 3*Tf^2, 4*Tf^3; 
         0, 1/2, (Tf/2)^2, (Tf/2)^3, (Tf/2)^4];

    b = [Zi; Viz; Zf; Vfz; zmax];

    x = A\b;

    trajectoire_pied_Z = x(5)*t.^4 + x(4)*t.^3 + x(3)*t.^2 + x(2)*t + x(1);
end
