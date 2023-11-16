% Numerical inverse kinematics with random target
% ik_random2.m
% 2004 Dec.17 s.kajita AIST

close all
clear              % claer work space
global uLINK   
global Trajx
global Trajy
global Trajz
global zmprefx
global zmprefy
global temps_un_pas 
global Ts

Ts = 0.01
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

clf
    DrawAllJoints(1);
    view(38,10)
    axis equal
    zlim([-0.2 1.2])
    grid on
for i=1:length(temps_un_pas)
%     uLINK(MP_BODY).p = [0.0, 0.0, 0.5]';
%     uLINK(MP_BODY).R = eye(3);
%     
    
%     Rfoot.p = p_ankle_r_int + [Trajx(i) Trajy(i) Trajz(i)]';
%     Rfoot.R = RPY2R(1/2*pi*(rand(3,1)-0.5));  %  -pi/4 < q < pi/4
%     rerr_norm = InverseKinematics(MP_ANKLE2_R, Rfoot);
%     
%     Lfoot.p = p_ankle_l_int + [Trajx(i) Trajy(i) Trajz(i)]';
%     Lfoot.R = RPY2R(1/2*pi*(rand(3,1)-0.5)); %  -pi/4 < q < pi/4
%     lerr_norm = InverseKinematics(MP_ANKLE2_L, Lfoot);
        
    clf
    DrawAllJoints(1);
    view(38,10)
    axis equal
    zlim([-0.2 1.2])
    grid on
    
    java.lang.Trhread.sleep(10)
end