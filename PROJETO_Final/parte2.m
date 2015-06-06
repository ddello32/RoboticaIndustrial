%% Parte 2 do projeto final

close all, clear all

%% Carrega robotics toolbox se preciso
if(~exist('rvctools', 'dir'))
    addpath('../rvctools')
    addpath('../rvctools/simulink')
end
if(~exist('SerialLink', 'class'))
    startup_rvc
end

%% Inicia robo
q0=[0 0 0 0 0 0 0];

L1=Link([0 12.4 0 pi/2 0 -pi/2]);
L2=Link([0 0 0 -pi/2 ]);
L3=Link([0 15.43 0 pi/2 ]);
L4=Link([0 0 0 -pi/2 0 0]);
L5=Link([0 15.925 0 pi/2]);
L6=Link([0 0 0 -pi/2 ]);
L7=Link([0 15.0 0 0 0 pi/2]);
bot=SerialLink([L1 L2 L3 L4 L5 L6 L7]);

%% Define trajetória deseijada
a=20; %horizontal radius
b=10; %vertical radius
O =[0, 20, 0]; %ellipse centre coordinates
t=-pi:0.05:pi;
x=x0+a*cos(t);
y=y0+b*sin(t);
ellipse = [y' O(2)*ones(1, length(t))' x'];
%% Calcula cinematica inversa
traj = zeros(length(t), 7);
pos = [eye(3,3) ellipse(1,:)'; 0 0 0 1];
traj(1,:) = bot.ikine(pos, q0, [1 1 1 0 0 0]);
for i = 2:length(t)
    pos = [eye(3,3) ellipse(i,:)'; 0 0 0 1];
    traj(i,:) = bot.ikine(pos, traj(i-1,:), [1 1 1 0 0 0]);
end
%% Plot
hold on
view([30, 45])
xlim([-30,30])
ylim([-20,20])
zlim([-30,30])
plot2(ellipse,'b')
for i=1: length(t)
    bot.plot(traj(i,:), 'noshading', 'notiles')
    atj=bot.fkine(traj(i,:));
    jta=transpose(atj);
    JTA(i,:)=jta(4,1:3);
    jta=JTA;
    plot2(jta(i,:),'ro')
end