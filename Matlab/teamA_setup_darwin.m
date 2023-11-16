%%% SetupBipedRobot.m
%%% Set biped robot structure of Figure 2.19, 2.20
%%% Field definition: Table 2.1 Link Parameters

global uLINK

ToDeg = 180/pi;
ToRad = pi/180;
UX = [1 0 0]';
UY = [0 1 0]';
UZ = [0 0 1]';
%%% a : Axe de rotation b : position selon le parent q: valeur du joint
uLINK    = struct('name','MP_BODY'    , 'm', 0.97559947, 'sister', 0, 'child', 2, 'b',[0  0  0.2195]','a',UZ,'q',0);

uLINK(2) = struct('name','MP_PELVIS_L' , 'm',  0.167, 'sister', 8, 'child', 3, 'b',[-0.005 0.037 -0.09355]'   ,'a',-UZ,'q',0);
uLINK(3) = struct('name','MP_THIGH1_L' , 'm',  0.11904336, 'sister', 0, 'child', 4, 'b',[0 0 -0.028652]'   ,'a',-UX,'q',0);
uLINK(4) = struct('name','MP_THIGH2_L' , 'm',  0.11904336, 'sister', 0, 'child', 5, 'b',[0 0 0]','a',-UY,'q',0);
uLINK(5) = struct('name','MP_TIBIA_L' , 'm',  0.070309794, 'sister', 0, 'child', 6, 'b',[0 0 -0.093]' ,'a',-UY,'q',0);
uLINK(6) = struct('name','MP_ANKLE1_L' , 'm',  0.16710792, 'sister', 0, 'child', 7, 'b',[ 0 0 -0.093]' ,'a',UY,'q',0);
uLINK(7) = struct('name','MP_ANKLE2_L' , 'm',  0.0794462, 'sister', 0, 'child', 0, 'b',[0  0   0]' ,'a',UX,'q',0);

uLINK(8) = struct('name','MP_PELVIS_R' , 'm',  0.197, 'sister', 0, 'child', 9, 'b',[-0.005 -0.037 -0.09355]'   ,'a',-UZ,'q',0);
uLINK(9) = struct('name','MP_THIGH1_R' , 'm',  0.11904336, 'sister', 0, 'child',10, 'b',[0 0 -0.028652]'   ,'a',-UX,'q',0);
uLINK(10)= struct('name','MP_THIGH2_R' , 'm',  0.11904336, 'sister', 0, 'child',11, 'b',[0 0 0]'   ,'a',UY,'q',0);
uLINK(11)= struct('name','MP_TIBIA_R' , 'm',  0.070309794, 'sister', 0, 'child',12, 'b',[ 0 0 -0.093]' ,'a',UY,'q',0);
uLINK(12)= struct('name','MP_ANKLE1_R' , 'm',  0.16710792, 'sister', 0, 'child',13, 'b',[ 0 0 -0.093]' ,'a',-UY,'q',0);
uLINK(13)= struct('name','MP_ANKLE2_R' , 'm',  0.0794462, 'sister', 0, 'child', 0, 'b',[0  0   0]' ,'a',UX,'q',0);

[uLINK(1).vertex,uLINK(1).face]   = MakeBox([0.1 0.3 0.5]  ,[0.05 0.15 -0.05] );    % BODY
[uLINK(7).vertex,uLINK(7).face]   = MakeBox([0.2 0.1 0.02] ,[0.05  0.05 0.05]);     % Foot
[uLINK(13).vertex,uLINK(13).face] = MakeBox([0.2 0.1 0.02] ,[0.05  0.05 0.05]);     % Foot

FindMother(1);   % Find mother link from sister and child data

%%% Substitute the ID to the link name variables. For example, BODY=1.
for n=1:length(uLINK)
    eval([uLINK(n).name,'=',num2str(n),';']);
end

uLINK(MP_BODY).p = [0.0, 0.0, 0.214625 + 0.143577]';
uLINK(MP_BODY).R = eye(3);
ForwardKinematics(1);

uLINK(MP_BODY).v = [0 0 0]';
uLINK(MP_BODY).w = [0 0 0]';
for n=1:length(uLINK)
    uLINK(n).dq     = 0;            % joitn speed   [rad/s]
end

clf
DrawAllJoints(1);
view(38,10)
axis equal
zlim([-0.2 1.2])
grid on