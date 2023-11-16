% Numerical inverse kinematics with random target
% ik_random2.m
% 2004 Dec.17 s.kajita AIST

close all
clear              % claer work space
global uLINK       % allow access from external functions

teamA_setup_darwin;   % Set the biped robot of Fig.2.19 and Fig.2.20

athigh = atan(0.4 / 0.093);
atibia = atan(0.4 / 0.093);



%%%%%%%%%%% set non singular posture@%%%%%%%%%%%%
uLINK(MP_THIGH1_L).q = 0.3*ToRad;
uLINK(MP_THIGH2_L).q = -athigh*ToRad;
uLINK(MP_TIBIA_L).q = (athigh+atibia)*ToRad;
uLINK(MP_ANKLE1_L).q = (atibia)*ToRad;
uLINK(MP_ANKLE2_L).q = (0.3)*ToRad;

uLINK(MP_THIGH1_R).q = 0.3*ToRad;
uLINK(MP_THIGH2_R).q = -athigh*ToRad;
uLINK(MP_TIBIA_R).q = (athigh+atibia)*ToRad;
uLINK(MP_ANKLE1_R).q = (atibia)*ToRad;
uLINK(MP_ANKLE2_R).q = (0.3)*ToRad;

uLINK(MP_BODY).p = [0.0, 0.0, 0.7]';
uLINK(MP_BODY).R = eye(3);

%%%%%%%%%%%random target foot position and orientation %%%%%%%%%%%%

rand('state',0);

figure
while 1
    uLINK(MP_BODY).p = [0.0, 0.0, 0.5]';
    uLINK(MP_BODY).R = eye(3);
    
    Rfoot.p = [0, -0.03, 0]' + 0.02*(rand(3,1)-0.5);
    Rfoot.R = RPY2R(1/2*pi*(rand(3,1)-0.5));  %  -pi/4 < q < pi/4
    rerr_norm = InverseKinematics(MP_ANKLE2_R, Rfoot);
    
    Lfoot.p = [0, -0.03, 0]' + 0.01*(rand(3,1)-0.5);
    Lfoot.R = RPY2R(1/2*pi*(rand(3,1)-0.5)); %  -pi/4 < q < pi/4
    lerr_norm = InverseKinematics(MP_ANKLE2_L, Lfoot);
        
    clf
    DrawAllJoints(1);
    view(38,10)
    axis equal
    zlim([-0.2 1.2])
    grid on
    
    fprintf('Right foot errorF %8.3e  Left foot error: %8.3e\n',rerr_norm,lerr_norm);
    fprintf('Type any key for another pose, Ctrl-C to abort\n');
    pause
end
