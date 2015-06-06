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

%% Define trajetória desejada
a=15; %raio em x
b=20; %raio em z
O =[0, -20, 20]; %cordenadas do centro da elipse
t=-pi:0.05:pi;
x=O(1)+a*cos(t);
z=O(3)+b*sin(t);
ellipse = [x' O(2)*ones(1, length(t))' z'];
%% Calcula cinematica inversa
qstart = bot.ikine([1 0 0 ellipse(1,1); 0 1 0 ellipse(1,2); 0 0 1 ellipse(1,3); 0 0 0 1], q0, [1 1 1 0 0 0]);
traj1 = jtraj(q0, qstart, 0:0.01:1);
traj = zeros(length(t), 7);
traj(1,:) = qstart;
for i = 2:length(t)
    pos = [eye(3,3) ellipse(i,:)'; 0 0 0 1];
    traj(i,:) = bot.ikine(pos, traj(i-1,:), [1 1 1 0 0 0]);
end
%% Plot
hold on
view([30, 45])
xlim([-40,40])
ylim([-20,40])
zlim([0,60])
plot2(ellipse,'b')
bot.plot(traj1, 'noshading', 'notiles')
for i=1: length(t)
    bot.plot(traj(i,:), 'noshading', 'notiles')
    atj=bot.fkine(traj(i,:));
    jta=transpose(atj);
    JTA(i,:)=jta(4,1:3);
    jta=JTA;
    plot2(jta(i,:),'ro')
end