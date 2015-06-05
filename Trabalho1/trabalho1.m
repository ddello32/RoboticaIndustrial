%% Load robotics toolbox
clc, clear all, close all
addpath('../rvctools')
startup_rvc
%% Smaller precision for easier visualization
old = digits(6);
%% Find transformation matrix
d1 = 1;
d2 = 2;
d6 = 1;
% We define pi as a symbolic in order to eliminate numeric errors in the
% calculation of our matrixes
syms d3 t1 t2 t4 t5 t6 pi;
T01 = (simplify(transl(0,0,d1)*trotz(t1+pi/2)*transl(0,0,0)*trotx(pi/2)));
T12 = (simplify(transl(0,0,d2)*trotz(t2+pi)*transl(0,0,0)*trotx(pi/2)));
T23 = (simplify(transl(0,0,d3)*trotz(pi)*transl(0,0,0)*trotx(pi/2)));
T03 = vpa(simplify(T01*T12*T23))
T34 = (simplify(transl(0,0,0)*trotz(t4+pi/2)*transl(0,0,0)*trotx(pi/2)));
T45 = (simplify(transl(0,0,0)*trotz(t5+pi/2)*transl(0,0,0)*trotx(pi/2)));
T56 = (simplify(transl(0,0,d6)*trotz(t6)*transl(0,0,0)*trotx(0)));
T36 = vpa(simplify(T34*T45*T56))
% Find the rotation matrixes
R03 = T03(1:3, 1:3)
R36 = T36(1:3, 1:3)
clear pi
%% Inverse kinematics
O = [-2.4339; 2.2803; 3.1476]
R = [-0.9010, 0, -0.4339;
    -0.3758, 0.5, 0.7803;
    0.2169, 0.8660, -0.4505]
Oc = O - d6*R*([0; 0; 1]);
xc = Oc(1);
yc = Oc(2);
zc = Oc(3);
teta1 = atan2(yc, xc) - atan2(-sqrt(xc^2 + yc^2 - d2^2), d2)
teta2 = pi/2 - atan2(zc - d1, sqrt(xc^2 + yc^2 - d2^2))
de3 = sqrt(xc^2 + yc^2 - d2^2 + (zc-d1)^2)
% Insert this values into R03 and R36 matrix
R03r = subs(R03, t1, teta1);
R03r = subs(R03r, t2, teta2);
R03r = subs(R03r, d3, de3);
Rmul = vpa(simplify(R03r'*R));
% From the equation R36 == Rmul, we can find an analytical solution 
% represented by the following code
if(Rmul(1,3) == 0 && Rmul(2,3) == 0)
    if(Rmul(3,3) ==1)
        teta5 = pi/2
        teta4 = 0
        teta6 = eval(atan2(Rmul(2,2),-Rmul(2,1)))
    else
        teta5 = -pi/2
        teta4 = eval(atan2(Rmul(2,2),Rmul(2,1)))
        teta6 = 0
    end
else
    teta5 = eval(atan2(Rmul(3,3),sqrt(1-Rmul(3,3)^2)))
    teta4 = eval(atan2(-Rmul(1,3),Rmul(2,3)))
    teta6 = eval(atan2(-Rmul(3,2),Rmul(3,1)))
end

%% Create model so we can visualize this robot
L1 = Link([0, d1, 0, pi/2, 0]) ;
L2 = Link([0, d2, 0, pi/2, 0]) ;
L3 = Link([pi, 0, 0, pi/2, 1]) ;
L4 = Link([0, 0, 0, pi/2, 0]) ;
L5 = Link([0, 0, 0, pi/2, 0]) ;
L6 = Link([0, d6, 0, 0, 0]) ;
L1.offset = pi/2;
L2.offset = pi;
L4.offset = pi/2;
L5.offset = pi/2;
bot = SerialLink([L1, L2, L3, L4, L5, L6], 'name', 'stanford')
bot.plotopt={'workspace', [-6 6 -6 6 -6 6]};
q0 = [0, 0, 0, 0, 0, 0]; % Offset of the initial state.
q = jtraj(q0, [teta1, teta2, de3, teta4, teta5, teta6]+q0, [0:0.05:2]');
%Show foward kinematics for the desired final position so we can verify if
%it is working correctly
foward_kin = bot.fkine([teta1, teta2, de3, teta4, teta5, teta6]+q0)
Diff = foward_kin - [R O;0 0 0 1]
bot.plot(q);
%% Old precision settings
digits(old)
%% Turn the plot into an animation
!convert -delay 5 animation/* animation.gif
%% Robot animation
%
% <<animation.gif>>
%