!%% Parte 2 do projeto final

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
a=20; %raio em x
b=28; %raio em y
O =[-10, -10, 0]; %cordenadas do centro da elipse
t=-pi:0.05:pi;
x=O(1)+a*cos(t);
y=O(2)+b*sin(t);
ellipse = [x' y' O(3)*ones(1, length(t))'];

a=10; %raio em x
b=20; %raio em y
O2 =[0, 20, 0]; %cordenadas do centro da elipse
t=-pi:0.05:pi;
x=O2(1)+a*cos(t);
y=O2(2)+b*sin(t);
ellipse2 = [x' O2(2)*ones(1, length(t))' y'];

% %% Charizard
% t=0:0.1:84*pi;
% x = ((-33./16.*sin(11./7-12.*t)-1./4.*sin(9./7-7.*t)-1./6.*sin(10./7-2.*t)+264./7.*sin(t+11./7)+17./2.*sin(3.*t+33./7)+4./3.*sin(4.*t+47./10)+17./11.*sin(5.*t+11./7)+19./11.*sin(6.*t+14./9)+3.*sin(8.*t+14./9)+9./4.*sin(9.*t+17./11)+1./9.*sin(10.*t+7./10)+29./9.*sin(11.*t+11./7)-5613./8).*heaviside(83.*pi-t).*heaviside(t-79.*pi)+(-29./7.*sin(11./7-10.*t)-1./7.*sin(13./9-9.*t)-91./8.*sin(11./7-6.*t)-1002./7.*sin(11./7-2.*t)-545./7.*sin(11./7-t)+175./4.*sin(3.*t+11./7)+91./45.*sin(4.*t+47./10)+51./4.*sin(5.*t+11./7)+25./4.*sin(7.*t+11./7)+1./2.*sin(8.*t+8./5)+10./7.*sin(11.*t+11./7)+19./8.*sin(12.*t+33./7)+4595./9).*heaviside(79.*pi-t).*heaviside(t-75.*pi)+(-37./8.*sin(14./9-4.*t)-30.*sin(11./7-2.*t)+28./5.*sin(t+11./7)+67./8.*sin(3.*t+11./7)+1./3.*sin(5.*t+3./2)+649./2).*heaviside(75.*pi-t).*heaviside(t-71.*pi)+(-89./10.*sin(11./7-2.*t)+206./5.*sin(t+11./7)+29./6.*sin(3.*t+11./7)+5./11.*sin(4.*t+47./10)+7./4.*sin(5.*t+11./7)-3972./13).*heaviside(71.*pi-t).*heaviside(t-67.*pi)+(-26./7.*sin(11./7-5.*t)-37./12.*sin(14./9-4.*t)-679./4.*sin(11./7-t)+195./7.*sin(2.*t+11./7)+16./5.*sin(3.*t+11./7)+10./3.*sin(6.*t+11./7)+3345./22).*heaviside(67.*pi-t).*heaviside(t-63.*pi)+(-36./7.*sin(14./9-9.*t)-175./16.*sin(14./9-7.*t)-79./6.*sin(11./7-5.*t)-71./8.*sin(11./7-4.*t)-776./21.*sin(11./7-3.*t)-287./13.*sin(11./7-2.*t)+616./9.*sin(t+11./7)+11./7.*sin(6.*t+33./7)+97./48.*sin(8.*t+8./5)+5./7.*sin(10.*t+23./15)+4./3.*sin(11.*t+47./10)+3./2.*sin(12.*t+8./5)-659./10).*heaviside(63.*pi-t).*heaviside(t-59.*pi)+(-2./5.*sin(7./5-30.*t)-6./11.*sin(7./6-26.*t)-3./2.*sin(7./5-25.*t)-8./5.*sin(10./7-24.*t)-32./11.*sin(26./17-18.*t)-49./9.*sin(3./2-16.*t)-25./9.*sin(16./11-15.*t)-11./5.*sin(14./9-10.*t)+91./23.*sin(t+61./13)+268./7.*sin(2.*t+11./7)+431./27.*sin(3.*t+47./10)+415./7.*sin(4.*t+8./5)+93./10.*sin(5.*t+13./8)+11./3.*sin(6.*t+33./7)+59./9.*sin(7.*t+5./3)+209./8.*sin(8.*t+8./5)+5./2.*sin(9.*t+12./7)+1./6.*sin(11.*t+23./5)+15./2.*sin(12.*t+13./8)+19./5.*sin(13.*t+17./10)+4./5.*sin(14.*t+13./8)+13./7.*sin(17.*t+3./2)+1./2.*sin(19.*t+17./4)+65./7.*sin(20.*t+18./11)+7./9.*sin(21.*t+7./3)+51./13.*sin(22.*t+5./3)+5./8.*sin(23.*t+9./5)+5./6.*sin(27.*t+33./7)+3./2.*sin(28.*t+12./7)+11./12.*sin(29.*t+16./9)-1497./4).*heaviside(59.*pi-t).*heaviside(t-55.*pi)+(-7./4.*sin(17./11-19.*t)-16./9.*sin(14./9-17.*t)-1./6.*sin(7./5-16.*t)-8./5.*sin(11./7-15.*t)-20./9.*sin(14./9-13.*t)-1./3.*sin(11./7-12.*t)-28./9.*sin(14./9-11.*t)-19./10.*sin(20./13-10.*t)-13./4.*sin(11./7-9.*t)-59./4.*sin(14./9-7.*t)-113./14.*sin(14./9-6.*t)+32./7.*sin(t+11./7)+49./9.*sin(2.*t+11./7)+405./29.*sin(3.*t+11./7)+37./6.*sin(4.*t+11./7)+82./5.*sin(5.*t+8./5)+19./7.*sin(8.*t+8./5)+3./7.*sin(14.*t+23./15)+16./15.*sin(18.*t+8./5)-3625./6).*heaviside(55.*pi-t).*heaviside(t-51.*pi)+(-65./11.*sin(11./7-5.*t)-87./8.*sin(11./7-3.*t)+917./11.*sin(t+11./7)+221./3.*sin(2.*t+11./7)+239./8.*sin(4.*t+11./7)+41./5.*sin(6.*t+11./7)-2018./11).*heaviside(51.*pi-t).*heaviside(t-47.*pi)+(-25./9.*sin(11./7-8.*t)+627./10.*sin(t+11./7)+1261./15.*sin(2.*t+11./7)+355./9.*sin(3.*t+11./7)+173./9.*sin(4.*t+33./7)+23./5.*sin(5.*t+11./7)+59./20.*sin(6.*t+3./2)+58./7.*sin(7.*t+11./7)+10./9.*sin(9.*t+37./8)+13./9.*sin(10.*t+23./15)+17./6.*sin(11.*t+17./11)+1./4.*sin(12.*t+11./7)-351./5).*heaviside(47.*pi-t).*heaviside(t-43.*pi)+(-5./4.*sin(11./7-12.*t)-1./4.*sin(3./2-11.*t)-9./4.*sin(11./7-10.*t)-7./2.*sin(11./7-8.*t)-12./5.*sin(14./9-6.*t)-94./5.*sin(11./7-4.*t)-7./5.*sin(14./9-3.*t)-126./5.*sin(11./7-2.*t)+2022./47.*sin(t+11./7)+1./3.*sin(5.*t+17./11)+13./9.*sin(7.*t+8./5)+2./5.*sin(9.*t+17./11)+5264./9).*heaviside(43.*pi-t).*heaviside(t-39.*pi)+(-44./3.*sin(11./7-6.*t)-37./7.*sin(20./13-4.*t)+281./9.*sin(t+11./7)+1223./12.*sin(2.*t+33./7)+657./16.*sin(3.*t+33./7)+134./9.*sin(5.*t+11./7)+19./18.*sin(7.*t+37./8)+53./6.*sin(8.*t+33./7)+20./7.*sin(9.*t+11./7)+4./7.*sin(10.*t+16./11)+13./8.*sin(11.*t+33./7)+35./9.*sin(12.*t+33./7)-20367./34).*heaviside(39.*pi-t).*heaviside(t-35.*pi)+(-16./9.*sin(17./11-12.*t)-5./6.*sin(3./2-11.*t)-29./8.*sin(11./7-6.*t)+1069./10.*sin(t+11./7)+78./5.*sin(2.*t+33./7)+53./5.*sin(3.*t+11./7)+64./21.*sin(4.*t+33./7)+10./9.*sin(5.*t+14./9)+59./11.*sin(7.*t+11./7)+16./9.*sin(8.*t+47./10)+7./4.*sin(9.*t+8./5)+1./9.*sin(10.*t+23./5)-13508./15).*heaviside(35.*pi-t).*heaviside(t-31.*pi)+(482./9.*sin(t+33./7)+696./5.*sin(2.*t+11./7)+27./2.*sin(3.*t+8./5)+29./4.*sin(4.*t+8./5)+40./11.*sin(5.*t+8./5)+76./7.*sin(6.*t+8./5)+9./5.*sin(7.*t+8./5)+20./7.*sin(8.*t+8./5)+7./3.*sin(9.*t+8./5)+64./21.*sin(10.*t+8./5)+1./2.*sin(11.*t+5./3)+13./5.*sin(12.*t+8./5)-791./6).*heaviside(31.*pi-t).*heaviside(t-27.*pi)+(1625./8.*sin(t+11./7)+38./11.*sin(2.*t+11./7)+110./7.*sin(3.*t+11./7)+13./8.*sin(4.*t+8./5)+41./8.*sin(5.*t+11./7)+9./10.*sin(6.*t+8./5)+25./7.*sin(7.*t+11./7)+3./4.*sin(8.*t+8./5)+16./7.*sin(9.*t+8./5)+6./13.*sin(10.*t+13./8)+5./4.*sin(11.*t+11./7)+1./9.*sin(12.*t+11./8)+5155./9).*heaviside(27.*pi-t).*heaviside(t-23.*pi)+(-19./2.*sin(11./7-6.*t)-29./15.*sin(3./2-3.*t)+1102./3.*sin(t+33./7)+141./7.*sin(2.*t+11./7)+75./2.*sin(4.*t+33./7)+277./9.*sin(5.*t+33./7)+1834./3).*heaviside(23.*pi-t).*heaviside(t-19.*pi)+(-3./5.*sin(6./13-14.*t)-1030./49.*sin(11./9-2.*t)+219./10.*sin(t+7./2)+139./6.*sin(3.*t+10./11)+71./10.*sin(4.*t+31./10)+233./29.*sin(5.*t+19./10)+40./11.*sin(6.*t+23./6)+61./9.*sin(7.*t+27./10)+23./6.*sin(8.*t+18./7)+21./8.*sin(9.*t+115./57)+11./9.*sin(10.*t+4./7)+23./10.*sin(11.*t+1./21)+7./4.*sin(12.*t+25./6)+3./4.*sin(13.*t+13./10)+4./9.*sin(15.*t+18./7)+1./3.*sin(16.*t+23./7)+2./7.*sin(17.*t+8./9)+6./13.*sin(18.*t+4)+1./13.*sin(19.*t+22./9)+1./5.*sin(20.*t+31./7)-4403./10).*heaviside(19.*pi-t).*heaviside(t-15.*pi)+(-13./14.*sin(16./11-23.*t)-11./8.*sin(3./7-15.*t)-3./4.*sin(13./10-14.*t)-18./7.*sin(15./11-13.*t)-2447./10.*sin(17./16-t)+181./7.*sin(2.*t+13./3)+194./9.*sin(3.*t+1./2)+18.*sin(4.*t+10./3)+25./9.*sin(5.*t+6./13)+74./9.*sin(6.*t+7./2)+58./11.*sin(7.*t+27./7)+11./8.*sin(8.*t+51./13)+7./3.*sin(9.*t+19./7)+11./3.*sin(10.*t+18./5)+5./3.*sin(11.*t+37./8)+24./7.*sin(12.*t+11./3)+10./9.*sin(16.*t+3./2)+15./8.*sin(17.*t+15./11)+19./10.*sin(18.*t+9./4)+11./6.*sin(19.*t+27./10)+2.*sin(20.*t+10./3)+10./7.*sin(21.*t+29./8)+22./15.*sin(22.*t+34./9)+3./8.*sin(24.*t+65./16)-938./5).*heaviside(15.*pi-t).*heaviside(t-11.*pi)+(-11./8.*sin(1./47-10.*t)-31./32.*sin(11./8-9.*t)-131./8.*sin(19./20-3.*t)+1651./22.*sin(t+3./4)+62./5.*sin(2.*t+14./15)+47./11.*sin(4.*t+7./5)+16./9.*sin(5.*t+8./7)+33./8.*sin(6.*t+24./7)+11./10.*sin(7.*t+8./9)+13./10.*sin(8.*t+33./7)+3./4.*sin(11.*t+27./13)+5./7.*sin(12.*t+27./10)-10999./26).*heaviside(11.*pi-t).*heaviside(t-7.*pi)+(-5./6.*sin(1./6-7.*t)-13./7.*sin(3./7-6.*t)+887./5.*sin(t+9./8)+79./6.*sin(2.*t+3./2)+227./12.*sin(3.*t+1./59)+59./7.*sin(4.*t+4./3)+9.*sin(5.*t+50./11)+13./9.*sin(8.*t+14./3)+2./3.*sin(9.*t+16./7)+3./2.*sin(10.*t+31./10)+27./10.*sin(11.*t+7./9)+17./10.*sin(12.*t+1./12)+12./7.*sin(13.*t+21./8)+29./30.*sin(14.*t+3./10)+3./4.*sin(15.*t+23./7)+5./7.*sin(16.*t+1./9)+1./7.*sin(17.*t+17./5)+3./7.*sin(18.*t+16./5)+1./4.*sin(19.*t+13./7)+1./6.*sin(20.*t+11./5)+1./2.*sin(21.*t+7./2)+1320./13).*heaviside(7.*pi-t).*heaviside(t-3.*pi)+(-1./2.*sin(1./6-78.*t)-5./9.*sin(7./9-76.*t)-4./11.*sin(10./7-64.*t)-6./5.*sin(7./6-56.*t)-12./7.*sin(47./46-54.*t)-5./3.*sin(1./5-52.*t)-11./5.*sin(4./11-49.*t)-5./2.*sin(11./8-36.*t)-31./7.*sin(13./10-30.*t)-83./6.*sin(6./7-14.*t)-326./5.*sin(3./4-7.*t)-287./3.*sin(5./6-6.*t)+23./15.*sin(50.*t)+4775./6.*sin(t+31./8)+406./11.*sin(2.*t+15./4)+457./6.*sin(3.*t+43./10)+471./5.*sin(4.*t+3./8)+1262./7.*sin(5.*t+7./4)+743./12.*sin(8.*t+11./5)+265./11.*sin(9.*t+16./9)+79./2.*sin(10.*t+21./5)+617./44.*sin(11.*t+17./4)+293./9.*sin(12.*t+10./3)+236./11.*sin(13.*t+3./2)+285./13.*sin(15.*t+23./10)+211./15.*sin(16.*t+61./13)+19./8.*sin(17.*t+21./5)+88./9.*sin(18.*t+16./9)+61./10.*sin(19.*t+11./7)+128./7.*sin(20.*t+13./10)+11.*sin(21.*t+7./5)+93./10.*sin(22.*t+17./7)+71./7.*sin(23.*t+11./7)+2.*sin(24.*t+19./8)+96./11.*sin(25.*t+5./3)+56./19.*sin(26.*t+17./8)+22./5.*sin(27.*t+11./8)+5./6.*sin(28.*t+17./6)+37./5.*sin(29.*t+23./6)+33./7.*sin(31.*t+40./9)+33./7.*sin(32.*t+123./31)+5./8.*sin(33.*t+9./5)+19./6.*sin(34.*t+13./4)+24./5.*sin(35.*t+13./6)+11./3.*sin(37.*t+16./9)+2./3.*sin(38.*t+5./6)+3./5.*sin(39.*t+5./8)+21./10.*sin(40.*t+13./10)+39./8.*sin(41.*t+13./6)+9./7.*sin(42.*t+13./7)+33./16.*sin(43.*t+9./5)+13./8.*sin(44.*t+12./7)+17./7.*sin(45.*t+11./3)+15./11.*sin(46.*t+25./9)+5./7.*sin(47.*t+1./8)+10./7.*sin(48.*t+58./13)+1./4.*sin(51.*t+1./8)+15./11.*sin(53.*t+5./3)+10./7.*sin(55.*t+43./10)+23./10.*sin(57.*t+21./5)+7./9.*sin(58.*t+41./10)+12./13.*sin(59.*t+18./5)+13./8.*sin(60.*t+67./22)+8./9.*sin(61.*t+38./15)+7./13.*sin(62.*t+81./20)+13./10.*sin(63.*t+18./7)+5./4.*sin(65.*t+23./5)+1./4.*sin(66.*t+3./5)+12./13.*sin(67.*t+32./7)+3./4.*sin(68.*t+26./7)+7./11.*sin(69.*t+17./5)+3./7.*sin(70.*t+23./7)+5./7.*sin(71.*t+29./8)+9./10.*sin(72.*t+27./8)+3./4.*sin(73.*t+8./3)+3./7.*sin(74.*t+26./9)+2./3.*sin(75.*t+43./21)+6./7.*sin(77.*t+9./2)+3./4.*sin(79.*t+50./11)-25./3).*heaviside(3.*pi-t).*heaviside(t+pi)).*heaviside(sqrt(sign(sin(t./2))));
% y = ((-32./11.*sin(11./7-9.*t)-23./8.*sin(11./7-4.*t)+4./11.*sin(t+13./9)+36./5.*sin(2.*t+11./7)+7./2.*sin(3.*t+11./7)+7./8.*sin(5.*t+14./9)+20./7.*sin(6.*t+14./9)+11./3.*sin(7.*t+14./9)+92./31.*sin(8.*t+11./7)+1./9.*sin(10.*t+22./9)+21./11.*sin(11.*t+14./3)+21./11.*sin(12.*t+47./10)+3333./5).*heaviside(83.*pi-t).*heaviside(t-79.*pi)+(-15./14.*sin(11./7-9.*t)+373./5.*sin(t+33./7)+706./5.*sin(2.*t+11./7)+251./5.*sin(3.*t+11./7)+32./5.*sin(4.*t+33./7)+97./8.*sin(5.*t+11./7)+31./3.*sin(6.*t+11./7)+177./22.*sin(7.*t+11./7)+7./4.*sin(8.*t+8./5)+19./4.*sin(10.*t+11./7)+16./7.*sin(11.*t+8./5)+3./8.*sin(12.*t+13./8)-10387./28).*heaviside(79.*pi-t).*heaviside(t-75.*pi)+(-91./11.*sin(14./9-5.*t)-8./5.*sin(19./13-4.*t)-179./8.*sin(14./9-3.*t)-3.*sin(14./9-2.*t)-3107./14.*sin(11./7-t)-2219./6).*heaviside(75.*pi-t).*heaviside(t-71.*pi)+(299./7.*sin(t+11./7)+80./9.*sin(2.*t+11./7)+19./5.*sin(3.*t+11./7)+2./3.*sin(4.*t+8./5)+15./8.*sin(5.*t+11./7)-1943./9).*heaviside(71.*pi-t).*heaviside(t-67.*pi)+(-43./6.*sin(11./7-6.*t)-6./7.*sin(23./15-5.*t)-128./9.*sin(11./7-4.*t)-722./7.*sin(11./7-2.*t)-339./5.*sin(11./7-t)+331./55.*sin(3.*t+11./7)-2905./9).*heaviside(67.*pi-t).*heaviside(t-63.*pi)+(-8./7.*sin(19./13-11.*t)-2./5.*sin(3./2-9.*t)-2./7.*sin(11./8-8.*t)-47./9.*sin(11./7-7.*t)-50./3.*sin(17./11-4.*t)-37./5.*sin(17./11-3.*t)+110./7.*sin(t+11./7)+329./3.*sin(2.*t+11./7)+4./7.*sin(5.*t+32./7)+16./5.*sin(6.*t+13./8)+29./6.*sin(10.*t+8./5)+1./2.*sin(12.*t+17./11)-6041./10).*heaviside(63.*pi-t).*heaviside(t-59.*pi)+(-5./8.*sin(9./7-30.*t)-7./6.*sin(10./7-29.*t)-2./9.*sin(43./42-28.*t)-4./7.*sin(11./8-27.*t)-24./11.*sin(14./9-20.*t)-123./19.*sin(22./15-18.*t)-15./4.*sin(19./13-17.*t)-31./4.*sin(14./9-12.*t)-101./7.*sin(14./9-8.*t)-103./8.*sin(11./7-3.*t)-24./7.*sin(3./2-2.*t)-425./6.*sin(11./7-t)+39./8.*sin(4.*t+33./7)+118./7.*sin(5.*t+8./5)+28./11.*sin(6.*t+5./3)+7./11.*sin(7.*t+17./13)+61./5.*sin(9.*t+8./5)+12./5.*sin(10.*t+14./3)+27./8.*sin(11.*t+14./9)+37./6.*sin(13.*t+8./5)+49./10.*sin(14.*t+18./11)+27./8.*sin(15.*t+47./10)+13./3.*sin(16.*t+8./5)+3./4.*sin(19.*t+6./5)+16./9.*sin(21.*t+17./10)+41./11.*sin(22.*t+17./10)+10./11.*sin(23.*t+11./6)+1./4.*sin(24.*t+15./7)+21./10.*sin(25.*t+5./3)+1./7.*sin(26.*t+8./3)-436./5).*heaviside(59.*pi-t).*heaviside(t-55.*pi)+(-7./9.*sin(17./11-18.*t)-sin(23./15-17.*t)-20./11.*sin(17./11-15.*t)-5./8.*sin(20./13-14.*t)-19./8.*sin(14./9-10.*t)-3./8.*sin(11./7-7.*t)-41./7.*sin(14./9-5.*t)-208./11.*sin(11./7-2.*t)-352./5.*sin(11./7-t)+23./2.*sin(3.*t+11./7)+271./16.*sin(4.*t+11./7)+86./5.*sin(6.*t+8./5)+5./8.*sin(8.*t+16./11)+23./12.*sin(9.*t+8./5)+4./9.*sin(11.*t+11./7)+5./6.*sin(12.*t+8./5)+7./8.*sin(13.*t+18./11)+1./25.*sin(16.*t+7./3)-724./5).*heaviside(55.*pi-t).*heaviside(t-51.*pi)+(-41./6.*sin(11./7-6.*t)-99./10.*sin(11./7-2.*t)-2023./9.*sin(11./7-t)+35./9.*sin(3.*t+11./7)+131./10.*sin(4.*t+33./7)+17./3.*sin(5.*t+11./7)+12583./27).*heaviside(51.*pi-t).*heaviside(t-47.*pi)+(-51./25.*sin(11./7-12.*t)-12./7.*sin(23./15-9.*t)+203./3.*sin(t+11./7)+323./5.*sin(2.*t+33./7)+119./4.*sin(3.*t+33./7)+17.*sin(4.*t+11./7)+58./3.*sin(5.*t+33./7)+29./7.*sin(6.*t+13./8)+223./16.*sin(7.*t+33./7)+15./4.*sin(8.*t+8./5)+11./5.*sin(10.*t+14./3)+9./7.*sin(11.*t+37./8)+1426./3).*heaviside(47.*pi-t).*heaviside(t-43.*pi)+(-2./3.*sin(23./15-12.*t)+19./5.*sin(t+18./11)+804./5.*sin(2.*t+11./7)+2./7.*sin(3.*t+9./5)+7./3.*sin(4.*t+51./11)+22./13.*sin(5.*t+14./9)+148./9.*sin(6.*t+11./7)+2./3.*sin(7.*t+37./8)+6./7.*sin(8.*t+14./3)+9./8.*sin(9.*t+11./7)+35./6.*sin(10.*t+11./7)+3./5.*sin(11.*t+37./8)+1238./3).*heaviside(43.*pi-t).*heaviside(t-39.*pi)+(-1./13.*sin(11./8-8.*t)+10./9.*sin(t+33./7)+201./8.*sin(2.*t+11./7)+31./9.*sin(3.*t+11./7)+13./4.*sin(4.*t+8./5)+113./15.*sin(5.*t+33./7)+22./5.*sin(6.*t+11./7)+1./3.*sin(7.*t+11./7)+17./5.*sin(9.*t+11./7)+25./26.*sin(10.*t+33./7)+6./5.*sin(11.*t+33./7)+4./5.*sin(12.*t+11./7)+6265./11).*heaviside(39.*pi-t).*heaviside(t-35.*pi)+(-3./5.*sin(7./5-12.*t)-58./19.*sin(14./9-11.*t)-71./18.*sin(14./9-9.*t)-63./10.*sin(14./9-6.*t)-37./8.*sin(14./9-5.*t)-19./4.*sin(14./9-4.*t)-64./7.*sin(11./7-3.*t)-230./9.*sin(11./7-2.*t)-445./4.*sin(11./7-t)+15./14.*sin(7.*t+11./7)+4./9.*sin(8.*t+14./9)+27./11.*sin(10.*t+11./7)+1606./5).*heaviside(35.*pi-t).*heaviside(t-31.*pi)+(-1./8.*sin(16./11-10.*t)-37./5.*sin(11./7-8.*t)-77./4.*sin(11./7-6.*t)-777./11.*sin(11./7-2.*t)-1988./13.*sin(11./7-t)+10./3.*sin(3.*t+11./7)+46./5.*sin(4.*t+11./7)+1./6.*sin(5.*t+6./5)+83./5.*sin(7.*t+11./7)+6./11.*sin(9.*t+14./9)+13./10.*sin(11.*t+8./5)+3./10.*sin(12.*t+13./8)-2345./8).*heaviside(31.*pi-t).*heaviside(t-27.*pi)+(-208./11.*sin(11./7-t)+889./24.*sin(2.*t+11./7)+9./5.*sin(3.*t+33./7)+307./22.*sin(4.*t+11./7)+9./7.*sin(5.*t+47./10)+27./7.*sin(6.*t+11./7)+5./6.*sin(7.*t+47./10)+7./6.*sin(8.*t+11./7)+7./6.*sin(9.*t+33./7)+11./9.*sin(10.*t+11./7)+1./6.*sin(11.*t+14./3)+3./2.*sin(12.*t+11./7)-4472./7).*heaviside(27.*pi-t).*heaviside(t-23.*pi)+(-150./11.*sin(11./7-6.*t)-66./7.*sin(14./9-4.*t)+1171./45.*sin(t+33./7)+2223./8.*sin(2.*t+33./7)+179./12.*sin(3.*t+11./7)+91./5.*sin(5.*t+11./7)+459).*heaviside(23.*pi-t).*heaviside(t-19.*pi)+(-17./5.*sin(4./9-10.*t)-114./7.*sin(9./8-3.*t)+279./14.*sin(t+12./7)+46./5.*sin(2.*t+37./11)+93./11.*sin(4.*t+9./7)+32./11.*sin(5.*t+1./8)+27./13.*sin(6.*t+19./9)+36./7.*sin(7.*t+4./3)+29./9.*sin(8.*t+9./5)+30./7.*sin(9.*t+3./5)+27./14.*sin(11.*t+31./7)+23./15.*sin(12.*t+7./3)+5./3.*sin(13.*t+3./7)+6./7.*sin(14.*t+35./8)+3./7.*sin(15.*t+7./5)+1./2.*sin(16.*t+17./8)+1./5.*sin(17.*t+15./7)+3./7.*sin(18.*t+7./4)+5./11.*sin(19.*t+9./5)+2./5.*sin(20.*t+51./11)+7576./11).*heaviside(19.*pi-t).*heaviside(t-15.*pi)+(-10./11.*sin(5./4-13.*t)-4./3.*sin(4./11-12.*t)-247./6.*sin(15./11-3.*t)+1149./10.*sin(t+29./7)+537./8.*sin(2.*t+9./4)+163./13.*sin(4.*t+1./5)+97./9.*sin(5.*t+53./15)+109./11.*sin(6.*t+33./10)+35./13.*sin(7.*t+15./4)+15./7.*sin(8.*t+17./6)+17./6.*sin(9.*t+69./17)+21./22.*sin(10.*t+4./7)+5./6.*sin(11.*t+38./13)+5./6.*sin(14.*t+16./5)+5./6.*sin(15.*t+43./13)+5./7.*sin(16.*t+3./5)+3./7.*sin(17.*t+1)+7./10.*sin(18.*t+3./5)+1./2.*sin(19.*t+13./9)+3./4.*sin(20.*t+35./18)+6./11.*sin(21.*t+17./6)+4./7.*sin(22.*t+16./7)+6./7.*sin(23.*t+23./7)+1./11.*sin(24.*t+29./7)-423./7).*heaviside(15.*pi-t).*heaviside(t-11.*pi)+(-1./2.*sin(7./9-9.*t)-159./5.*sin(7./6-t)+22./3.*sin(2.*t+2./7)+157./9.*sin(3.*t+35./8)+15./4.*sin(4.*t+16./9)+15./8.*sin(5.*t+19./10)+49./12.*sin(6.*t+35./13)+1./3.*sin(7.*t+10./3)+7./5.*sin(8.*t+31./7)+sin(10.*t+5./8)+3./7.*sin(11.*t+23./11)+sin(12.*t+19./7)-14111./17).*heaviside(11.*pi-t).*heaviside(t-7.*pi)+(-4./7.*sin(1-19.*t)-17./11.*sin(20./19-13.*t)-9./4.*sin(6./5-11.*t)-104./7.*sin(6./5-3.*t)-471./8.*sin(13./10-t)+55./6.*sin(2.*t+6./5)+27./11.*sin(4.*t+3./4)+215./7.*sin(5.*t+22./5)+23./5.*sin(6.*t+59./17)+14./3.*sin(7.*t+1./5)+11./8.*sin(8.*t+23./8)+15./7.*sin(9.*t+13./6)+7./4.*sin(10.*t+31./7)+45./44.*sin(12.*t+21./10)+11./12.*sin(14.*t+5./3)+27./26.*sin(15.*t+14./5)+6./7.*sin(16.*t+113./28)+1./2.*sin(17.*t+10./3)+10./11.*sin(18.*t+21./8)+4./7.*sin(20.*t+82./27)+8./11.*sin(21.*t+13./3)-9646./11).*heaviside(7.*pi-t).*heaviside(t-3.*pi)+(-7./11.*sin(1./6-70.*t)-2./5.*sin(2./3-69.*t)-3./5.*sin(1./19-68.*t)-1./2.*sin(3./8-66.*t)-1./6.*sin(1-64.*t)-9./7.*sin(11./12-63.*t)-8./5.*sin(1./7-62.*t)-sin(1./7-57.*t)-35./18.*sin(9./8-56.*t)-12./7.*sin(1./8-51.*t)-10./3.*sin(1./3-50.*t)-sin(3./10-48.*t)-16./15.*sin(5./4-45.*t)-19./6.*sin(4./9-43.*t)-33./10.*sin(4./7-42.*t)-7./6.*sin(7./9-39.*t)-10./9.*sin(11./7-37.*t)-26./7.*sin(20./13-33.*t)-178./59.*sin(5./4-28.*t)-25./9.*sin(5./6-23.*t)-92./9.*sin(10./7-20.*t)-34./3.*sin(8./11-15.*t)-104./5.*sin(4./5-10.*t)-106./11.*sin(12./13-9.*t)-3232./5.*sin(4./5-t)+1635./7.*sin(2.*t+1./6)+224./3.*sin(3.*t+3./4)+1319./11.*sin(4.*t+7./15)+1664./11.*sin(5.*t+25./6)+606./7.*sin(6.*t+8./5)+253./9.*sin(7.*t+12./5)+359./7.*sin(8.*t+8./7)+40./11.*sin(11.*t+7./5)+820./21.*sin(12.*t+27./11)+139./11.*sin(13.*t+25./7)+113./10.*sin(14.*t+39./10)+57./5.*sin(16.*t+9./4)+203./17.*sin(17.*t+2./5)+35./6.*sin(18.*t+7./9)+16.*sin(19.*t+23./7)+11./4.*sin(21.*t+9./7)+13./7.*sin(22.*t+17./7)+69./4.*sin(24.*t+27./13)+49./9.*sin(25.*t+56./13)+37./12.*sin(26.*t+9./4)+3./2.*sin(27.*t+23./8)+351./50.*sin(29.*t+9./5)+30./7.*sin(30.*t+28./11)+18./5.*sin(31.*t+9./5)+46./7.*sin(32.*t+4./7)+43./9.*sin(34.*t+5./6)+62./21.*sin(35.*t+4./7)+7./4.*sin(36.*t+22./15)+20./7.*sin(38.*t+13./6)+5./7.*sin(40.*t+13./3)+32./9.*sin(41.*t+3./4)+22./15.*sin(44.*t+16./17)+8./5.*sin(46.*t+17./7)+11./4.*sin(47.*t+27./7)+5./3.*sin(49.*t+37./8)+26./7.*sin(52.*t+9./4)+14./9.*sin(53.*t+17./5)+22./7.*sin(54.*t+43./22)+3./2.*sin(55.*t+9./2)+2./5.*sin(58.*t+1./2)+8./7.*sin(59.*t+9./7)+15./16.*sin(60.*t+4./7)+26./27.*sin(61.*t+16./9)+12./7.*sin(65.*t+17./4)+36./35.*sin(67.*t+56./13)+13./12.*sin(71.*t+15./4)+15./14.*sin(72.*t+28./9)+4./7.*sin(73.*t+40./9)+14./13.*sin(74.*t+16./7)+3./5.*sin(75.*t+14./3)+10./7.*sin(76.*t+21./5)+18./11.*sin(77.*t+40./13)+7./11.*sin(78.*t+32./7)+12./11.*sin(79.*t+29./11)+374./7).*heaviside(3.*pi-t).*heaviside(t+pi)).*heaviside(sqrt(sign(sin(t./2))));
% charizard = [0.03*x' 0.03*y' O(3)*ones(1, length(t))'];
% %% Pikachu
%  t=0:0.05:52*pi;
%  x = O(1) + 0.05*((-1./4.*sin(10./7-23.*t)-3./10.*sin(4./3-22.*t)-2./5.*sin(7./5-19.*t)-1./5.*sin(7./5-16.*t)-3./7.*sin(10./7-15.*t)-3./8.*sin(13./9-9.*t)-19./13.*sin(11./7-3.*t)+222./5.*sin(t+11./7)+41./2.*sin(2.*t+11./7)+34./9.*sin(4.*t+11./7)+1./3.*sin(5.*t+8./5)+3./8.*sin(6.*t+8./5)+12./7.*sin(7.*t+13./8)+11./7.*sin(8.*t+13./8)+1./4.*sin(10.*t+20./13)+2./9.*sin(11.*t+16./9)+3./8.*sin(12.*t+8./5)+1./3.*sin(13.*t+7./4)+1./2.*sin(14.*t+17./10)+5./7.*sin(17.*t+17./10)+1./28.*sin(18.*t+9./2)+1./2.*sin(20.*t+12./7)+3./7.*sin(21.*t+16./9)+6./11.*sin(24.*t+7./4)-979./9).*heaviside(51.*pi-t).*heaviside(t-47.*pi)+(-6./5.*sin(14./9-22.*t)-1./9.*sin(7./5-19.*t)-9./8.*sin(14./9-18.*t)-1./14.*sin(15./11-15.*t)-6./5.*sin(11./7-12.*t)-7./6.*sin(11./7-8.*t)-29./10.*sin(11./7-6.*t)-104./3.*sin(11./7-2.*t)+415./18.*sin(t+11./7)+71./18.*sin(3.*t+11./7)+19./8.*sin(4.*t+33./7)+22./21.*sin(5.*t+8./5)+3./8.*sin(7.*t+61./13)+5./9.*sin(9.*t+11./7)+1./8.*sin(10.*t+14./3)+4./7.*sin(11.*t+11./7)+4./11.*sin(13.*t+14./3)+1./7.*sin(14.*t+14./3)+2./7.*sin(16.*t+5./3)+1./6.*sin(17.*t+5./3)+6./7.*sin(20.*t+8./5)+1./7.*sin(21.*t+5./3)+1./6.*sin(23.*t+8./5)-2765./8).*heaviside(47.*pi-t).*heaviside(t-43.*pi)+(1189./22.*sin(t+11./7)+3./4.*sin(2.*t+13./8)+11./2.*sin(3.*t+8./5)+2./7.*sin(4.*t+17./7)+22./9.*sin(5.*t+18./11)+1./4.*sin(6.*t+17./7)+16./17.*sin(7.*t+20./11)+1./5.*sin(8.*t+29./9)-1627./7).*heaviside(43.*pi-t).*heaviside(t-39.*pi)+(-3./7.*sin(1./18-5.*t)-3./4.*sin(1./2-3.*t)+109./9.*sin(t+13./10)+5./8.*sin(2.*t+11./3)+5./9.*sin(4.*t+10./3)+3./10.*sin(6.*t+21./8)+2./9.*sin(7.*t+2./3)+1./4.*sin(8.*t+23./8)-1190./9).*heaviside(39.*pi-t).*heaviside(t-35.*pi)+(188./21.*sin(t+27./28)+2./5.*sin(2.*t+17./6)+2./3.*sin(3.*t+91./23)+3./8.*sin(4.*t+53./18)+2./11.*sin(5.*t+1./7)-369).*heaviside(35.*pi-t).*heaviside(t-31.*pi)+(-8./9.*sin(1./10-12.*t)-34./9.*sin(10./9-6.*t)-137./10.*sin(5./7-2.*t)+26./5.*sin(t+13./4)+118./5.*sin(3.*t+11./8)+43./8.*sin(4.*t+13./7)+49./6.*sin(5.*t+11./12)+22./5.*sin(7.*t+13./4)+17./16.*sin(8.*t+1./7)+5./4.*sin(9.*t+1./4)+5./7.*sin(10.*t+17./5)+29./15.*sin(11.*t+5./6)-1915./8).*heaviside(31.*pi-t).*heaviside(t-27.*pi)+(-2./7.*sin(10./7-7.*t)-sin(1./27-4.*t)+68./7.*sin(t+44./15)+76./9.*sin(2.*t+37./10)+30./7.*sin(3.*t+1)+8./9.*sin(5.*t+3./2)+4./5.*sin(6.*t+31./8)+3./7.*sin(8.*t+10./3)+6./13.*sin(9.*t+8./7)+1./3.*sin(10.*t+31./9)-2135./9).*heaviside(27.*pi-t).*heaviside(t-23.*pi)+(-3./8.*sin(1./4-23.*t)-3./5.*sin(1./8-22.*t)-13./8.*sin(5./4-20.*t)-9./7.*sin(3./2-16.*t)-41./5.*sin(4./3-4.*t)+768./7.*sin(t+11./5)+109./5.*sin(2.*t+16./7)+150./13.*sin(3.*t+11./6)+33./7.*sin(5.*t+97./24)+23./4.*sin(6.*t+5./7)+69./7.*sin(7.*t+9./8)+32./5.*sin(8.*t+21./5)+7./6.*sin(9.*t+22./9)+28./5.*sin(10.*t+5./6)+43./10.*sin(11.*t+26./7)+14./9.*sin(12.*t+5./11)+13./9.*sin(13.*t+40./9)+11./6.*sin(14.*t+2./5)+3./2.*sin(15.*t+17./10)+7./11.*sin(17.*t+4./3)+3./8.*sin(18.*t+31./10)+4./7.*sin(19.*t+14./9)+6./5.*sin(21.*t+17./7)+4./7.*sin(24.*t+27./8)+1006./11).*heaviside(23.*pi-t).*heaviside(t-19.*pi)+(-63./8.*sin(2./7-8.*t)-38./13.*sin(11./9-6.*t)-14./5.*sin(1./17-4.*t)+77./9.*sin(t+1./2)+52./7.*sin(2.*t+10./3)+22./9.*sin(3.*t+76./17)+21./8.*sin(5.*t+26./7)+3.*sin(7.*t+15./8)+64./7.*sin(9.*t+57./14)+6.*sin(10.*t+17./6)-544./7).*heaviside(19.*pi-t).*heaviside(t-15.*pi)+(-37./10.*sin(4./7-5.*t)-3.*sin(3./7-3.*t)+24./7.*sin(t+7./6)+9./7.*sin(2.*t+2./5)+31./15.*sin(4.*t+37./8)+9./5.*sin(6.*t+12./5)+59./12.*sin(7.*t+13./6)+15./7.*sin(8.*t+25./8)+134./15.*sin(9.*t+7./3)+73./8.*sin(10.*t+1./5)-4406./11).*heaviside(15.*pi-t).*heaviside(t-11.*pi)+(236./7.*sin(t+6./5)+1./2.*sin(2.*t+47./12)-627./5).*heaviside(11.*pi-t).*heaviside(t-7.*pi)+(69./2.*sin(t+5./6)-715./2).*heaviside(7.*pi-t).*heaviside(t-3.*pi)+(-19./9.*sin(6./5-21.*t)-37./10.*sin(7./9-19.*t)-23./8.*sin(1-17.*t)-16./3.*sin(7./6-16.*t)-29./5.*sin(1./5-9.*t)-919./11.*sin(1./7-3.*t)+1573./6.*sin(t+91./45)+214./5.*sin(2.*t+33./8)+421./14.*sin(4.*t+13./8)+61./6.*sin(5.*t+19./5)+401./16.*sin(6.*t+43./14)+511./51.*sin(7.*t+35./8)+144./7.*sin(8.*t+5./6)+137./10.*sin(10.*t+25./13)+18./7.*sin(11.*t+15./7)+17./9.*sin(12.*t+41./9)+9./7.*sin(13.*t+13./7)+29./10.*sin(14.*t+22./7)+25./8.*sin(15.*t+1./4)+12./5.*sin(18.*t+11./8)+14./5.*sin(20.*t+27./7)+13./8.*sin(22.*t+12./7)+7./6.*sin(23.*t+7./9)+26./11.*sin(24.*t+23./7)-1891./8).*heaviside(3.*pi-t).*heaviside(t+pi)).*heaviside(sqrt(sign(sin(t./2))));
%  y = O(2) + 0.05*((-8./11.*sin(11./8-22.*t)-1./2.*sin(10./7-21.*t)+67./6.*sin(t+33./7)+1478./29.*sin(2.*t+11./7)+3./5.*sin(3.*t+30./7)+26./3.*sin(4.*t+11./7)+1./6.*sin(5.*t+13./9)+30./29.*sin(6.*t+8./5)+2./5.*sin(7.*t+14./3)+88./29.*sin(8.*t+8./5)+1./4.*sin(9.*t+31./7)+11./8.*sin(10.*t+8./5)+1./16.*sin(11.*t+9./2)+1./12.*sin(12.*t+5./4)+1./10.*sin(13.*t+25./11)+11./8.*sin(14.*t+18./11)+2./7.*sin(15.*t+37./8)+1./6.*sin(16.*t+11./8)+2./9.*sin(17.*t+5./3)+1./5.*sin(18.*t+17./10)+1./13.*sin(19.*t+19./8)+23./24.*sin(20.*t+12./7)+7./11.*sin(23.*t+9./5)+9./7.*sin(24.*t+7./4)-1538./7).*heaviside(51.*pi-t).*heaviside(t-47.*pi)+(-2./7.*sin(20./13-23.*t)-1./6.*sin(3./2-20.*t)-5./7.*sin(20./13-17.*t)-1./9.*sin(20./13-11.*t)-1./6.*sin(13./9-9.*t)-19./6.*sin(17./11-3.*t)+263./5.*sin(t+11./7)+614./15.*sin(2.*t+11./7)+87./10.*sin(4.*t+11./7)+1./7.*sin(5.*t+11./8)+19./11.*sin(6.*t+11./7)+7./5.*sin(7.*t+11./7)+4./3.*sin(8.*t+8./5)+9./5.*sin(10.*t+14./9)+4./7.*sin(12.*t+8./5)+3./11.*sin(13.*t+3./2)+1./8.*sin(14.*t+22./15)+1./9.*sin(15.*t+12./7)+6./5.*sin(16.*t+11./7)+2./9.*sin(18.*t+11./7)+3./5.*sin(19.*t+8./5)+1./26.*sin(21.*t+15./11)+6./7.*sin(22.*t+8./5)-1867./8).*heaviside(47.*pi-t).*heaviside(t-43.*pi)+(118./39.*sin(t+11./7)+40./7.*sin(2.*t+33./7)+49./25.*sin(3.*t+14./3)+12./5.*sin(4.*t+8./5)+1./9.*sin(5.*t+32./13)+5./2.*sin(6.*t+13./8)+2./5.*sin(7.*t+22./5)+3./4.*sin(8.*t+7./4)-143./10).*heaviside(43.*pi-t).*heaviside(t-39.*pi)+(-1./8.*sin(2./3-8.*t)-1./2.*sin(7./5-2.*t)-246./19.*sin(1./7-t)+1./4.*sin(3.*t+33./16)+1./6.*sin(4.*t+17./6)+1./5.*sin(5.*t+31./7)+1./11.*sin(6.*t+50./17)+1./8.*sin(7.*t+30./7)+665./6).*heaviside(39.*pi-t).*heaviside(t-35.*pi)+(-119./10.*sin(7./15-t)+2./11.*sin(2.*t+25./7)+2./9.*sin(3.*t+5./8)+1./5.*sin(4.*t+33./7)+1./4.*sin(5.*t+19./10)+1023./10).*heaviside(35.*pi-t).*heaviside(t-31.*pi)+(-1./7.*sin(2./7-12.*t)-1./8.*sin(3./10-5.*t)+25./7.*sin(t+77./17)+355./59.*sin(2.*t+41./40)+27./5.*sin(3.*t+46./15)+33./7.*sin(4.*t+11./3)+27./10.*sin(6.*t+13./9)+5./11.*sin(7.*t+11./5)+5./8.*sin(8.*t+3)+8./5.*sin(9.*t+16./15)+16./15.*sin(10.*t+1./7)+7./9.*sin(11.*t+12./5)-862./7).*heaviside(31.*pi-t).*heaviside(t-27.*pi)+(-1./3.*sin(5./4-8.*t)-2./5.*sin(5./9-7.*t)-5./7.*sin(11./8-5.*t)-7./2.*sin(15./14-2.*t)+29./8.*sin(t+41./10)+11./6.*sin(3.*t+13./3)+7./6.*sin(4.*t+1./27)+2./7.*sin(6.*t+8./7)+1./9.*sin(9.*t+9./5)+2./7.*sin(10.*t+1./10)+201./5).*heaviside(27.*pi-t).*heaviside(t-23.*pi)+(-4./11.*sin(8./9-12.*t)-10./7.*sin(19./13-10.*t)+623./3.*sin(t+10./7)+39./5.*sin(2.*t+10./11)+251./9.*sin(3.*t+4./3)+5./7.*sin(4.*t+4./3)+61./6.*sin(5.*t+4./3)+14./9.*sin(6.*t+23./7)+76./25.*sin(7.*t+9./7)+3./4.*sin(8.*t+1./4)+19./5.*sin(9.*t+3./2)+17./6.*sin(11.*t+6./5)+13./8.*sin(13.*t+14./13)+8./9.*sin(14.*t+17./6)+24./25.*sin(15.*t+1./2)+1./6.*sin(16.*t+13./8)+5./8.*sin(17.*t+1)+1./7.*sin(18.*t+18./17)+6./7.*sin(19.*t+1)+1./4.*sin(20.*t+4./9)+2./7.*sin(21.*t+7./5)+1./3.*sin(22.*t+8./7)+2./5.*sin(23.*t+1./26)+2./11.*sin(24.*t+8./7)-243./8).*heaviside(23.*pi-t).*heaviside(t-19.*pi)+(-111./10.*sin(4./5-9.*t)-12./5.*sin(7./13-2.*t)+1./6.*sin(t+48./11)+13./8.*sin(3.*t+27./7)+71./24.*sin(4.*t+6./11)+22./9.*sin(5.*t+7./2)+19./7.*sin(6.*t+8./17)+20./7.*sin(7.*t+34./9)+55./7.*sin(8.*t+6./5)+64./9.*sin(10.*t+38./9)+27./5).*heaviside(19.*pi-t).*heaviside(t-15.*pi)+(-22./7.*sin(4./3-8.*t)-19./7.*sin(20./13-6.*t)+38./13.*sin(t+1./24)+12./11.*sin(2.*t+5./9)+26./7.*sin(3.*t+7./9)+11./5.*sin(4.*t+12./11)+37./10.*sin(5.*t+17./10)+51./10.*sin(7.*t+10./3)+33./4.*sin(9.*t+26./7)+41./5.*sin(10.*t+9./5)-27./2).*heaviside(15.*pi-t).*heaviside(t-11.*pi)+(-172./5.*sin(3./8-t)+5./4.*sin(2.*t+7./2)+2303./24).*heaviside(11.*pi-t).*heaviside(t-7.*pi)+(441./5-455./12.*sin(7./9-t)).*heaviside(7.*pi-t).*heaviside(t-3.*pi)+(-1./3.*sin(1./20-18.*t)-7./5.*sin(7./9-17.*t)-18./11.*sin(2./5-14.*t)-24./5.*sin(1./13-9.*t)+2767./7.*sin(t+11./3)+229./5.*sin(2.*t+17./7)+313./8.*sin(3.*t+22./5)+32./3.*sin(4.*t+22./5)+169./6.*sin(5.*t+21./8)+23./7.*sin(6.*t+26./11)+21./2.*sin(7.*t+5./6)+55./6.*sin(8.*t+14./5)+212./13.*sin(10.*t+24./7)+26./9.*sin(11.*t+9./2)+16./5.*sin(12.*t+25./6)+35./17.*sin(13.*t+4./11)+15./8.*sin(15.*t+7./10)+2./3.*sin(16.*t+20./9)+16./7.*sin(19.*t+4./5)+13./7.*sin(20.*t+29./7)+14./3.*sin(21.*t+7./5)+4./3.*sin(22.*t+7./4)+12./7.*sin(23.*t+34./33)+7./4.*sin(24.*t+27./7)-211./5).*heaviside(3.*pi-t).*heaviside(t+pi)).*heaviside(sqrt(sign(sin(t./2))));
%  pikachu = [x' y' O(3)*ones(1, length(t))'];
% %% Unicornio
% t = 0:0.01:2*pi;
% x = O(1) + 0.05*(-12./25.*sin(1./10-48.*t)-72./13.*sin(11./7-42.*t)-8./3.*sin(13./9-40.*t)-13./12.*sin(7./13-37.*t)-73./18.*sin(3./5-32.*t)-68./9.*sin(7./10-28.*t)-19./4.*sin(2./5-26.*t)-96./5.*sin(3./10-18.*t)-4.*sin(4./3-17.*t)-171./7.*sin(7./9-10.*t)-243./5.*sin(23./24-9.*t)-54./5.*sin(5./14-6.*t)-478./7.*sin(11./8-2.*t)+5944./13.*sin(t+27./8)+824./9.*sin(3.*t+12./11)+1312./9.*sin(4.*t+47./12)+1448./15.*sin(5.*t+9./5)+667./16.*sin(7.*t+24./13)+297./8.*sin(8.*t+27./7)+419./21.*sin(11.*t+22./13)+105./8.*sin(12.*t+31./9)+117./7.*sin(13.*t+17./7)+86./3.*sin(14.*t+29./7)+18.*sin(15.*t+32./11)+185./12.*sin(16.*t+17./8)+124./13.*sin(19.*t+4)+83./8.*sin(20.*t+3./7)+37./3.*sin(21.*t+3./11)+73./6.*sin(22.*t+27./10)+148./11.*sin(23.*t+4./3)+45./7.*sin(24.*t+25./11)+12.*sin(25.*t+5./4)+49./12.*sin(27.*t+11./7)+79./9.*sin(29.*t+2)+43./10.*sin(30.*t+3./2)+26./7.*sin(31.*t+3./5)+15./4.*sin(33.*t+1./16)+46./13.*sin(34.*t+9./17)+35./12.*sin(35.*t+39./11)+57./10.*sin(36.*t+46./11)+31./12.*sin(38.*t+31./9)+71./13.*sin(39.*t+1./13)+23./13.*sin(41.*t+16./7)+62./25.*sin(43.*t+13./7)+8./11.*sin(44.*t+19./14)+30./11.*sin(45.*t+17./10)+35./11.*sin(46.*t+9./10)+7./4.*sin(47.*t+21./11)+1./2.*sin(49.*t+11./4)+14./13.*sin(50.*t+51./19)+35./36.*sin(51.*t+29./10)+21./10.*sin(52.*t+19./12)+11./8.*sin(53.*t+17./5)+15./7.*sin(54.*t+47./11)+10./9.*sin(55.*t+3./8)+11./5.*sin(56.*t+27./11)+10./7.*sin(57.*t+40./13)+13./19.*sin(58.*t+24./23)+32./9.*sin(59.*t+21./13)+9./5.*sin(60.*t+33./7)+2303./18);
% y = O(2) + 0.05*(-6./11.*sin(7./5-55.*t)-4./13.*sin(5./4-52.*t)-12./7.*sin(5./9-49.*t)-4./7.*sin(7./12-47.*t)-4.*sin(3./8-42.*t)-33./7.*sin(1./2-32.*t)-23./6.*sin(5./4-22.*t)-16./7.*sin(4./7-21.*t)-113./12.*sin(6./5-20.*t)-111./8.*sin(5./9-19.*t)-46./9.*sin(4./5-17.*t)-159./5.*sin(13./9-5.*t)-1768./7.*sin(6./7-t)+1841./23.*sin(2.*t+49./13)+398./11.*sin(3.*t+17./7)+163./4.*sin(4.*t+29./15)+485./11.*sin(6.*t+42./43)+205./9.*sin(7.*t+17./6)+873./19.*sin(8.*t+3./5)+615./14.*sin(9.*t+20./11)+397./7.*sin(10.*t+18./7)+104./7.*sin(11.*t+32./9)+53./6.*sin(12.*t+51./11)+139./8.*sin(13.*t+23./5)+262./9.*sin(14.*t+15./7)+149./13.*sin(15.*t+11./3)+55./8.*sin(16.*t+1./2)+129./8.*sin(18.*t+4./5)+140./13.*sin(23.*t+30./7)+60./11.*sin(24.*t+14./5)+15./4.*sin(25.*t+6./5)+19./13.*sin(26.*t+17./4)+34./11.*sin(27.*t+5./2)+244./27.*sin(28.*t+44./13)+91./9.*sin(29.*t+24./11)+22./13.*sin(30.*t+29./8)+79./13.*sin(31.*t+1./8)+16./5.*sin(33.*t+57./13)+7./6.*sin(34.*t+22./5)+7./2.*sin(35.*t+17./8)+17./10.*sin(36.*t+3./13)+15./16.*sin(37.*t+15./4)+79./17.*sin(38.*t+35./11)+16./9.*sin(39.*t+1./18)+23./12.*sin(40.*t+13./10)+21./8.*sin(41.*t+9./5)+33./10.*sin(43.*t+52./21)+12./7.*sin(44.*t+17./8)+56./19.*sin(45.*t+4)+57./10.*sin(46.*t+17./7)+29./8.*sin(48.*t+58./13)+19./11.*sin(50.*t+40./11)+23./11.*sin(51.*t+9./4)+10./9.*sin(53.*t+17./8)+13./10.*sin(54.*t+13./8)+122./41.*sin(56.*t+116./39)+16./9.*sin(57.*t+32./13)+sin(58.*t+53./12)+13./6.*sin(59.*t+7./3)+9./7.*sin(60.*t+2./3)-399./8);
% unicorn = [x' y' O(3)*ones(1, length(t))'];
%% Calcula cinematica inversa da primeira ellipse
entrada = ellipse;
qstart = bot.ikine([1 0 0 entrada(1,1); 0 1 0 entrada(1,2); 0 0 1 entrada(1,3); 0 0 0 1], q0, [1 1 1 0 0 0]);
traj1 = jtraj(q0, qstart, 0:0.05:1);
traj = zeros(length(entrada), 7);
traj(1,:) = qstart;
for i = 2:length(entrada)
    pos = [eye(3,3) entrada(i,:)'; 0 0 0 1];
    traj(i,:) = bot.ikine(pos, traj(i-1,:), [1 1 1 0 0 0]);
end
%% Calcula cinematica inversa da segunda ellipse
entrada2 = ellipse2;
qstart2 = bot.ikine([1 0 0 entrada2(1,1); 0 1 0 entrada2(1,2); 0 0 1 entrada2(1,3); 0 0 0 1], traj(end,:), [1 1 1 0 0 0]);
traj2 = zeros(length(entrada2), 7);
traj2(1,:) = qstart2;
for i = 2:length(t)
    pos = [eye(3,3) entrada2(i,:)'; 0 0 0 1];
    traj2(i,:) = bot.ikine(pos, traj2(i-1,:), [1 1 1 0 0 0]);
end
trajlink = jtraj(traj(end,:), traj2(1,:), 0:0.05:1);
%% Plot
figure,
hold on
view([30, 45])
xlim([-40,20])
ylim([-40,20])
zlim([0,60])
plot2(entrada,'b')
plot2(entrada2,'b')
bot.plot(traj1, 'noshading', 'notiles')
for i=1: length(traj)
    bot.plot(traj(i,:), 'noshading', 'notiles')
    atj=bot.fkine(traj(i,:));
    plot2(atj(1:3,4)','ro')
end
bot.plot(trajlink, 'noshading', 'notiles')
for i=1: length(traj2)
    bot.plot(traj2(i,:), 'noshading', 'notiles')
    atj=bot.fkine(traj2(i,:));
    plot2(atj(1:3,4)','ro')
end

% %% Create gif
% % Create a new figure
% figure();
% set(gcf, 'Renderer', 'zbuffer');
% hold on
% 
% numFrames = length(traj1) + length(traj) + length(trajlink) + length(traj2); 
% % Plot first frame, create color map and allocate image 4D array
% view([30, 45])
% xlim([-40,20])
% ylim([-40,20])
% zlim([0,60])
% plot2(entrada,'b')
% plot2(entrada2,'b')
% bot.plot(traj1(1,:), 'noshading', 'notiles')
% frame = getframe(gcf);
% [images, map] = rgb2ind(frame.cdata, 256, 'nodither');
% images(1, 1, 1, numFrames) = 0;
%  
% for n= 2:length(traj1)
%     bot.plot(traj1(n,:), 'noshading', 'notiles')
%     frame = getframe(gcf);
%     images(:, :, 1, n) = rgb2ind(frame.cdata, map, 'nodither');
% end
% 
% for i=1: length(traj)
%     bot.plot(traj(i,:), 'noshading', 'notiles')
%     atj=bot.fkine(traj(i,:));
%     plot2(atj(1:3,4)','ro')
%     frame = getframe(gcf);
%     images(:, :, 1, i+length(traj1)) = rgb2ind(frame.cdata, map, 'nodither');
% end
% 
% for n= 1:length(trajlink)
%     bot.plot(trajlink(n,:), 'noshading', 'notiles')
%     frame = getframe(gcf);
%     images(:, :, 1, n+length(traj1)+length(traj)) = rgb2ind(frame.cdata, map, 'nodither');
% end
% for i=1: length(traj2)
%     bot.plot(traj2(i,:), 'noshading', 'notiles')
%     atj=bot.fkine(traj2(i,:));
%     plot2(atj(1:3,4)','ro')
%     frame = getframe(gcf);
%     images(:, :, 1, i+length(traj1)+length(traj)+length(trajlink)) = rgb2ind(frame.cdata, map, 'nodither');
% 
% end
% imwrite(images, map, 'ellipses.gif', 'DelayTime', 0, 'LoopCount', inf);