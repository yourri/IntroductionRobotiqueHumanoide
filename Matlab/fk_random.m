close all
clear              
global uLINK      

teamA_setup_darwin;   

rand('state',0);  
figure
while 1
    qR1 = 2/3*pi*(rand(6,1)-0.5);  %  -pi/2 < q < pi/2
    qR1(4) = pi*rand;          %   0 < q4 < pi 
    
    qL1 = pi*(rand(6,1)-0.5);  %  -pi/2 < q < pi/2
    qL1(4) = pi*rand;          %   0 < q4 < pi 
    
    for n=0:5
        uLINK(MP_ANKLE2_R+n).q = qR1(n+1);
        uLINK(MP_ANKLE2_L+n).q = qL1(n+1);
    end
    
    uLINK(MP_BODY).p = [0.0, 0.0, 0.7]';
    uLINK(MP_BODY).R = eye(3);
    ForwardKinematics(1);
    
    clf
    DrawAllJoints(1);
    view(38,14)
    axis equal
    zlim([0.1 1.3])
    grid on
    
    fprintf('Ctrl-C:終了, それ以外：別の姿勢を表示\n');
    pause
end
