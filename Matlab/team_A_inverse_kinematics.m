close all
clear all             % clear work space
global uLINK   
global single_support
global double_support
global Ts
global comx
global comy
global dX

Ts = 0.01;
temps_initial = 2;
teamA_setup_darwin; 
team_A_ZMP_trajectory;
athigh = atan(0.04 / 0.093);
atibia = atan(0.04 / 0.093);

team_A_trajectoire_pied;

uLINK(MP_THIGH1_L).q = 0;
uLINK(MP_THIGH2_L).q = athigh;
uLINK(MP_TIBIA_L).q = -(athigh+atibia);
uLINK(MP_ANKLE1_L).q = -(atibia);
uLINK(MP_ANKLE2_L).q = (0.0);

uLINK(MP_THIGH1_R).q = 0;
uLINK(MP_THIGH2_R).q = -athigh;
uLINK(MP_TIBIA_R).q = (athigh+atibia);
uLINK(MP_ANKLE1_R).q = (atibia);
uLINK(MP_ANKLE2_R).q = (0.0);



uLINK(MP_BODY).p = [0.0, 0.0, 0.30]';
uLINK(MP_BODY).R = eye(3);


ForwardKinematics(1);
index = 1;
premier_pas = 10;
for i=1: temps_initial/Ts
    A = [uLINK(MP_PELVIS_R).q uLINK(MP_THIGH1_R).q uLINK(MP_THIGH2_R).q uLINK(MP_TIBIA_R).q uLINK(MP_ANKLE1_R).q uLINK(MP_ANKLE2_R).q uLINK(MP_PELVIS_L).q uLINK(MP_THIGH1_L).q uLINK(MP_THIGH2_L).q uLINK(MP_TIBIA_L).q uLINK(MP_ANKLE1_L).q uLINK(MP_ANKLE2_L).q]; 
    writematrix(A, '/mnt/home/usager/robm2725/Simulation_rob_humanoide/Darwin_Gazebo_GEI845/Darwin_Gazebo/src/darwin_control/src/test.txt', 'WriteMode', 'append', 'Delimiter', '\t');
end
trajectoire_pied_x_total_gauche = [];
trajectoire_pied_x_total_droit = [];
while index < (single_support/Ts + double_support/Ts) * 8
    % double support
    for i=1:(double_support / Ts) 
        ForwardKinematics(1);
        p_ankle_r_int = uLINK(MP_ANKLE2_R).p;
        p_ankle_l_int = uLINK(MP_ANKLE2_L).p;
        uLINK(MP_BODY).p = [comx(index) comy(index) uLINK(MP_BODY).p(3)]';
        Rfoot.p = [p_ankle_r_int(1) p_ankle_r_int(2) p_ankle_r_int(3)]';
        Rfoot.R = RPY2R([0, 0, 0]); 
        rerr_normr = InverseKinematics_CL(MP_ANKLE2_R, Rfoot);
        Lfoot.p = [p_ankle_l_int(1) p_ankle_l_int(2) p_ankle_l_int(3)]';
        Lfoot.R = RPY2R([0, 0, 0]);
        rerr_norm1 = InverseKinematics_CL(MP_ANKLE2_L, Lfoot) ;
        index = index + 1;
        afficher
        A = [uLINK(MP_PELVIS_R).q uLINK(MP_THIGH1_R).q uLINK(MP_THIGH2_R).q uLINK(MP_TIBIA_R).q uLINK(MP_ANKLE1_R).q uLINK(MP_ANKLE2_R).q uLINK(MP_PELVIS_L).q uLINK(MP_THIGH1_L).q uLINK(MP_THIGH2_L).q uLINK(MP_TIBIA_L).q uLINK(MP_ANKLE1_L).q uLINK(MP_ANKLE2_L).q];
        writematrix(A, '/mnt/home/usager/robm2725/Simulation_rob_humanoide/Darwin_Gazebo_GEI845/Darwin_Gazebo/src/darwin_control/src/test.txt', 'WriteMode', 'append', 'Delimiter', '\t');
        
        if length(trajectoire_pied_x_total_gauche) > 0
            trajectoire_pied_x_total_gauche = [trajectoire_pied_x_total_gauche trajectoire_pied_x_total_gauche(end)];
            trajectoire_pied_x_total_droit = [trajectoire_pied_x_total_droit trajectoire_pied_x_total_droit(end)];
        else
            trajectoire_pied_x_total_gauche = [trajectoire_pied_x_total_gauche 0];
            trajectoire_pied_x_total_droit = [trajectoire_pied_x_total_droit 0];
        end
    end
    
    
    ForwardKinematics(1);
    p_ankle_r_int = uLINK(MP_ANKLE2_R).p;
    p_ankle_l_int = uLINK(MP_ANKLE2_L).p;
    p_body = uLINK(MP_BODY).p;
    % pas droit single support
    if premier_pas == 10
        trajectoire_pied_droit_x = creation_trajectoire_pied_x(p_ankle_r_int(1), p_ankle_r_int(1) + dX, 0, single_support, Ts);
        premier_pas = 4132;
    else 
        trajectoire_pied_droit_x = creation_trajectoire_pied_x(p_ankle_r_int(1), p_ankle_r_int(1) + dX * 2, 0, single_support, Ts);
    end
%     
%     figure
%     plot(trajectoire_pied_droit_x)
%     pause 
    trajectoire_pied_droit_z = creation_trajectoire_pied_z(p_ankle_r_int(3), p_ankle_r_int(3), 0, single_support, Ts);
    for i=1:single_support / Ts + 1
        ForwardKinematics(1);
        p_ankle_r_int = uLINK(MP_ANKLE2_R).p;
        p_ankle_l_int = uLINK(MP_ANKLE2_L).p;
        uLINK(MP_BODY).p = [comx(index)+0.015 comy(index) uLINK(MP_BODY).p(3)]';
        Rfoot.p = [trajectoire_pied_droit_x(i) p_ankle_r_int(2) trajectoire_pied_droit_z(i)']';
        Rfoot.R = RPY2R([0, 0, 0]); 
        rerr_normr = InverseKinematics_CL(MP_ANKLE2_R, Rfoot);
        Lfoot.p = [p_ankle_l_int(1) p_ankle_l_int(2) p_ankle_l_int(3)]';
        Lfoot.R = RPY2R([0, 0, 0]);  %  -pi/4 < q < pi/4
        rerr_norm1 = InverseKinematics_CL(MP_ANKLE2_L, Lfoot) ;
        index = index + 1;
        afficher
        A = [uLINK(MP_PELVIS_R).q uLINK(MP_THIGH1_R).q uLINK(MP_THIGH2_R).q uLINK(MP_TIBIA_R).q uLINK(MP_ANKLE1_R).q uLINK(MP_ANKLE2_R).q uLINK(MP_PELVIS_L).q uLINK(MP_THIGH1_L).q uLINK(MP_THIGH2_L).q uLINK(MP_TIBIA_L).q uLINK(MP_ANKLE1_L).q uLINK(MP_ANKLE2_L).q]; 
        writematrix(A, '/mnt/home/usager/robm2725/Simulation_rob_humanoide/Darwin_Gazebo_GEI845/Darwin_Gazebo/src/darwin_control/src/test.txt', 'WriteMode', 'append', 'Delimiter', '\t');
        trajectoire_pied_x_total_gauche = [trajectoire_pied_x_total_gauche trajectoire_pied_x_total_gauche(end)];
        trajectoire_pied_x_total_droit = [trajectoire_pied_x_total_droit trajectoire_pied_droit_x(i)];
    end
    
    % double support
    for i=1:double_support / Ts
        ForwardKinematics(1);
        p_ankle_r_int = uLINK(MP_ANKLE2_R).p;
        p_ankle_l_int = uLINK(MP_ANKLE2_L).p;
        p_body = uLINK(MP_BODY).p;
        uLINK(MP_BODY).p = [comx(index)+0.015 comy(index) uLINK(MP_BODY).p(3)]';
        Rfoot.p = [p_ankle_r_int(1) p_ankle_r_int(2) p_ankle_r_int(3)]';
        Rfoot.R = RPY2R([0, 0, 0]); 
        rerr_normr = InverseKinematics_CL(MP_ANKLE2_R, Rfoot);
        Lfoot.p = [p_ankle_l_int(1) p_ankle_l_int(2) p_ankle_l_int(3)]';
        Lfoot.R = RPY2R([0, 0, 0]);
        rerr_norm1 = InverseKinematics_CL(MP_ANKLE2_L, Lfoot) ;
        index = index + 1;
        afficher
        A = [uLINK(MP_PELVIS_R).q uLINK(MP_THIGH1_R).q uLINK(MP_THIGH2_R).q uLINK(MP_TIBIA_R).q uLINK(MP_ANKLE1_R).q uLINK(MP_ANKLE2_R).q uLINK(MP_PELVIS_L).q uLINK(MP_THIGH1_L).q uLINK(MP_THIGH2_L).q uLINK(MP_TIBIA_L).q uLINK(MP_ANKLE1_L).q uLINK(MP_ANKLE2_L).q]; 
        writematrix(A, '/mnt/home/usager/robm2725/Simulation_rob_humanoide/Darwin_Gazebo_GEI845/Darwin_Gazebo/src/darwin_control/src/test.txt', 'WriteMode', 'append', 'Delimiter', '\t');
        trajectoire_pied_x_total_gauche = [trajectoire_pied_x_total_gauche trajectoire_pied_x_total_gauche(end)];
        trajectoire_pied_x_total_droit = [trajectoire_pied_x_total_droit trajectoire_pied_x_total_gauche(end)];
    end
    
    % pas gauche single support
    trajectoire_pied_gauche_x = creation_trajectoire_pied_x(p_ankle_l_int(1), p_ankle_l_int(1) + dX * 2, 0, single_support, Ts);
    trajectoire_pied_gauche_z = creation_trajectoire_pied_z(p_ankle_l_int(3), p_ankle_l_int(3), 0, single_support, Ts);
    for i=1:single_support / Ts
        ForwardKinematics(1);
        p_ankle_r_int = uLINK(MP_ANKLE2_R).p;
        p_ankle_l_int = uLINK(MP_ANKLE2_L).p;
        p_body = uLINK(MP_BODY).p;
        uLINK(MP_BODY).p = [comx(index)+0.015 comy(index) uLINK(MP_BODY).p(3)]';
        Lfoot.p = [trajectoire_pied_gauche_x(i) p_ankle_l_int(2) trajectoire_pied_gauche_z(i)']';
        Lfoot.R = RPY2R([0, 0, 0]); 
        rerr_norml = InverseKinematics_CL(MP_ANKLE2_L, Lfoot);
        Rfoot.p = [p_ankle_r_int(1) p_ankle_r_int(2) p_ankle_r_int(3)]';
        Rfoot.R = RPY2R([0, 0, 0]);  %  -pi/4 < q < pi/4
        rerr_normr = InverseKinematics_CL(MP_ANKLE2_R, Rfoot) ;
        index = index + 1;
        afficher
        A = [uLINK(MP_PELVIS_R).q uLINK(MP_THIGH1_R).q uLINK(MP_THIGH2_R).q uLINK(MP_TIBIA_R).q uLINK(MP_ANKLE1_R).q uLINK(MP_ANKLE2_R).q uLINK(MP_PELVIS_L).q uLINK(MP_THIGH1_L).q uLINK(MP_THIGH2_L).q uLINK(MP_TIBIA_L).q uLINK(MP_ANKLE1_L).q uLINK(MP_ANKLE2_L).q]; 
        writematrix(A, '/mnt/home/usager/robm2725/Simulation_rob_humanoide/Darwin_Gazebo_GEI845/Darwin_Gazebo/src/darwin_control/src/test.txt', 'WriteMode', 'append', 'Delimiter', '\t');
        trajectoire_pied_x_total_gauche = [trajectoire_pied_x_total_gauche trajectoire_pied_gauche_x(i)];
        trajectoire_pied_x_total_droit = [trajectoire_pied_x_total_droit trajectoire_pied_x_total_gauche(end)];
    end
end


% pas gauche single support
trajectoire_pied_gauche_x = creation_trajectoire_pied_x(p_ankle_l_int(1), p_ankle_l_int(1) + dX, 0, single_support, Ts);
trajectoire_pied_gauche_z = creation_trajectoire_pied_z(p_ankle_l_int(3), p_ankle_l_int(3), 0, single_support, Ts);
for i=1:single_support / Ts
    if  index > length(comx)
        break
    end
    ForwardKinematics(1);
    p_ankle_r_int = uLINK(MP_ANKLE2_R).p;
    p_ankle_l_int = uLINK(MP_ANKLE2_L).p;
    p_body = uLINK(MP_BODY).p;
    uLINK(MP_BODY).p = [comx(index)+0.015 comy(index) uLINK(MP_BODY).p(3)]';
    Lfoot.p = [trajectoire_pied_gauche_x(i) p_ankle_l_int(2) trajectoire_pied_gauche_z(i)']';
    Lfoot.R = RPY2R([0, 0, 0]); 
    rerr_norml = InverseKinematics_CL(MP_ANKLE2_L, Lfoot);
    Rfoot.p = [p_ankle_r_int(1) p_ankle_r_int(2) p_ankle_r_int(3)]';
    Rfoot.R = RPY2R([0, 0, 0]);  %  -pi/4 < q < pi/4
    rerr_normr = InverseKinematics_CL(MP_ANKLE2_R, Rfoot);
    index = index + 1;
    afficher
    A = [uLINK(MP_PELVIS_R).q uLINK(MP_THIGH1_R).q uLINK(MP_THIGH2_R).q uLINK(MP_TIBIA_R).q uLINK(MP_ANKLE1_R).q uLINK(MP_ANKLE2_R).q uLINK(MP_PELVIS_L).q uLINK(MP_THIGH1_L).q uLINK(MP_THIGH2_L).q uLINK(MP_TIBIA_L).q uLINK(MP_ANKLE1_L).q uLINK(MP_ANKLE2_L).q]; 
    writematrix(A, '/mnt/home/usager/robm2725/Simulation_rob_humanoide/Darwin_Gazebo_GEI845/Darwin_Gazebo/src/darwin_control/src/test.txt', 'WriteMode', 'append', 'Delimiter', '\t');
            trajectoire_pied_x_total_gauche = [trajectoire_pied_x_total_gauche trajectoire_pied_gauche_x(i)];
        trajectoire_pied_x_total_droit = [trajectoire_pied_x_total_droit trajectoire_pied_x_total_gauche(end)];
end
% 
% figure
% %plot(tplot, trajectoire_pied_x_total_gauche)
% hold on
% plot(tplot, trajectoire_pied_x_total_droit)
% plot(tplot, comx)
% plot(tplot, zmprefx)
% legend("gauche", "droit", "com", "zmprefx")

function afficher
%     clf
%     DrawAllJoints(1);
%     view(0,90)
%     axis equal
%     zlim([-0.2 1.2])
%     grid on
%     fprintf('Type any key for another pose, Ctrl-C to abort\n');    
%     pause(0.1)
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
    zmax = 0.03;

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
