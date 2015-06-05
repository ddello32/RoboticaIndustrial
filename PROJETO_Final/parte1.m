%% Parte 1 do projeto final

close all, clear all

%% Load robotic toolbox if necessary
if(~exist('rvctools', 'dir'))
    addpath('../rvctools')
    addpath('../rvctools/simulink')
end
if(~exist('SerialLink', 'class'))
    startup_rvc
end

%% Simulacoes em malha aberta seção 6.3
% Carrega o robo, utilizamos o modelo sem perdas por atrito seco entre os links
% para acelerar e simplificar os calculos, uma vez que esse atrito pode
% introduzir problemas numéricos.
mdl_puma560;
bot = p560.nofriction();
% Posição desejada
qdes = [0 pi/2 -pi/2 0 0 0];
% Calcula o torque nescessário para manter a posição desejada com
% velocidades angulares e acelerações nulas
T = bot.rne(qdes, zeros(1,6), zeros(1,6));

% Carrega o modelo simulink
model = 'parte1simulink';
load_system(model);


%% Simulacao 1
q0 = [0 0 0 0 0 0]; %Posicao inicial
sim(model);
sim1 = figure;
plot(states);
xlabel('Tempo(s)')
ylabel('Angulo(rad)')
title('Resposta de q')
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6');
saveas(sim1,'sim1.eps','epsc')
bot.plot(states.Data, 'delay', 5/length(states.Time));
%% Simulacao 2
q0 = [0 pi -pi/2 0 0 0]; %Posicao inicial
sim(model);
sim2 = figure;
plot(states);
xlabel('Tempo(s)')
ylabel('Angulo(rad)')
title('Resposta de q')
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6');
saveas(sim2,'sim2.eps','epsc')
figure,
bot.plot(states.Data, 'delay', 5/length(states.Time));
%% Simulacao 3
q0 = [0 pi/2 -pi/2 0 0 0]; %Posicao inicial
sim(model);
sim3 = figure;
plot(states);
xlabel('Tempo(s)')
ylabel('Angulo(rad)')
title('Resposta de q')
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6');
saveas(sim3,'sim3.eps','epsc')
bot.plot(states.Data, 'delay', 5/length(states.Time));
%% Simulacao 4
q0 = [0 pi/2+0.05 -pi/2 0 0 0]; %Posicao inicial
sim(model);
sim4 = figure;
plot(states);
xlabel('Tempo(s)')
ylabel('Angulo(rad)')
title('Resposta de q')
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6');
saveas(sim4,'sim4.eps','epsc')
bot.plot(states.Data, 'delay', 5/length(states.Time));
%% Simulacoes usando o ode45
%% Simulacao 1
q0 = [0 0 0 0 0 0]; %Posicao inicial
[time, qdyn1, qddyn] = bot.fdyn(5, T, q0, zeros(1,6))
sim1ode = figure;
plot(time, qdyn1);
xlabel('Tempo(s)')
ylabel('Angulo(rad)')
title('Resposta de q')
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6');
saveas(sim1ode,'sim1ode.eps','epsc')
figure,
bot.plot(qdyn1, 'delay', 5/length(time))
%% Simulacao 2
q0 = [0 pi -pi/2 0 0 0]; %Posicao inicial
[time, qdyn2, qddyn] = bot.fdyn(5, T, q0, zeros(1,6))
sim2ode = figure;
plot(time, qdyn2);
xlabel('Tempo(s)')
ylabel('Angulo(rad)')
title('Resposta de q')
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6');
saveas(sim2ode,'sim2ode.eps','epsc')
figure,
bot.plot(qdyn2, 'delay', 5/length(time))
%% Simulacao 3
q0 = [0 pi/2 -pi/2 0 0 0]; %Posicao inicial
[time, qdyn3, qddyn] = bot.fdyn(5, T, q0, zeros(1,6))
sim3ode = figure;
plot(time, qdyn3);
xlabel('Tempo(s)')
ylabel('Angulo(rad)')
title('Resposta de q')
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6');
saveas(sim3ode,'sim3ode.eps','epsc')
figure,
bot.plot(qdyn3, 'delay', 5/length(time))
%% Simulacao 4
q0 = [0 pi/2+0.05 -pi/2 0 0 0]; %Posicao inicial
[time, qdyn4, qddyn] = p560.nofriction().fdyn(5, T, q0, zeros(1,6))
sim4ode = figure;
plot(time, qdyn4);
xlabel('Tempo(s)')
ylabel('Angulo(rad)')
title('Resposta de q')
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6');
saveas(sim4ode,'sim4ode.eps','epsc')
figure,
bot.plot(qdyn4, 'delay', 5/length(time))
%% Simulacoes em malha aberta seção 6.4
T = [0 0 0 0 0 0];
%% Sem atrito seco e viscoso
%% Simulacao 1
q0 = [0 0 0 0 0 0]; %Posicao inicial
[time, qdyn, qddyn] = p560.nofriction('all').fdyn(5, T, q0, zeros(1,6));
sime1ode = figure;
plot(time, qdyn);
xlabel('Tempo(s)')
ylabel('Angulo(rad)')
title('Resposta de q')
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6');
saveas(sime1ode,'sime1ode.eps','epsc')
sime1kin = figure;
m = p560.nofriction('all').inertia(qdyn);
e = zeros(1, length(qdyn));
for i = 1:length(qdyn)
    e(i) = 1/2*qddyn(i,:)*m(:,:,i)*qddyn(i,:)';
end
plot(time, e);
xlabel('Tempo(s)')
ylabel('Energia(J)')
title('Energia cinetica do sistema')
saveas(sime1kin,'sime1kin.eps','epsc')
figure,
p560.plot(qdyn, 'delay', 5/length(time))

%% Simulacao 2
q0 = [0 0.000001 0 0 0 0]; %Posicao inicial
[time, qdyn, qddyn] = p560.nofriction('all').fdyn(5, T, q0, zeros(1,6));
sime2ode = figure;
plot(time, qdyn);
xlabel('Tempo(s)')
ylabel('Angulo(rad)')
title('Resposta de q')
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6');
saveas(sime2ode,'sime2ode.eps','epsc')
sime2kin = figure;
m = p560.nofriction('all').inertia(qdyn);
e = zeros(1, length(qdyn));
for i = 1:length(qdyn)
    e(i) = 1/2*qddyn(i,:)*m(:,:,i)*qddyn(i,:)';
end
plot(time, e);
xlabel('Tempo(s)')
ylabel('Energia(J)')
title('Energia cinetica do sistema')
saveas(sime2kin,'sime2kin.eps','epsc')
figure,
p560.plot(qdyn, 'delay', 5/length(time))

%% Simulacao 3
q0 = qs; %Posicao inicial
[time, qdyn, qddyn] = p560.nofriction('all').fdyn(5, T, q0, zeros(1,6));
sime3ode = figure;
plot(time, qdyn);
xlabel('Tempo(s)')
ylabel('Angulo(rad)')
title('Resposta de q')
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6');
saveas(sime3ode,'sime3ode.eps','epsc')
sime3kin = figure;
m = p560.nofriction('all').inertia(qdyn);
e = zeros(1, length(qdyn));
for i = 1:length(qdyn)
    e(i) = 1/2*qddyn(i,:)*m(:,:,i)*qddyn(i,:)';
end
plot(time, e);
xlabel('Tempo(s)')
ylabel('Energia(J)')
title('Energia cinetica do sistema')
saveas(sime3kin,'sime3kin.eps','epsc')
figure,
p560.plot(qdyn, 'delay', 5/length(time))

%% Com atrito
%% Simulacao 1
q0 = [0 0 0 0 0 0]; %Posicao inicial
[time, qdyn, qddyn] = bot.fdyn(5, T, q0, zeros(1,6));
sime1ode = figure;
plot(time, qdyn);
xlabel('Tempo(s)')
ylabel('Angulo(rad)')
title('Resposta de q')
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6');
saveas(sime1ode,'sime1ode.eps','epsc')
sime1kin = figure;
m = p560.inertia(qdyn);
e = zeros(1, length(qdyn));
for i = 1:length(qdyn)
    e(i) = 1/2*qddyn(i,:)*m(:,:,i)*qddyn(i,:)';
end
plot(time, e);
xlabel('Tempo(s)')
ylabel('Energia(J)')
title('Energia cinetica do sistema')
saveas(sime1kin,'sime1kin.eps','epsc')
figure,
bot.plot(qdyn, 'delay', 5/length(time))

%% Simulacao 2
q0 = [0 0.000001 0 0 0 0]; %Posicao inicial
[time, qdyn, qddyn] = p560.fdyn(5, T, q0, zeros(1,6));
sime2ode = figure;
plot(time, qdyn);
xlabel('Tempo(s)')
ylabel('Angulo(rad)')
title('Resposta de q')
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6');
saveas(sime2ode,'sime2ode.eps','epsc')
sime2kin = figure;
m = p560.inertia(qdyn);
e = zeros(1, length(qdyn));
for i = 1:length(qdyn)
    e(i) = 1/2*qddyn(i,:)*m(:,:,i)*qddyn(i,:)';
end
plot(time, e);
xlabel('Tempo(s)')
ylabel('Energia(J)')
title('Energia cinetica do sistema')
saveas(sime2kin,'sime2kin.eps','epsc')
figure,
bot.plot(qdyn, 'delay', 5/length(time))

%% Simulacao 3
q0 = qs; %Posicao inicial
[time, qdyn, qddyn] = p560.fdyn(5, T, q0, zeros(1,6));
sime3ode = figure;
plot(time, qdyn);
xlabel('Tempo(s)')
ylabel('Angulo(rad)')
title('Resposta de q')
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6');
saveas(sime3ode,'sime3ode.eps','epsc')
sime3kin = figure;
m = p560.inertia(qdyn);
e = zeros(1, length(qdyn));
for i = 1:length(qdyn)
    e(i) = 1/2*qddyn(i,:)*m(:,:,i)*qddyn(i,:)';
end
plot(time, e);
xlabel('Tempo(s)')
ylabel('Energia(J)')
title('Energia cinetica do sistema')
saveas(sime3kin,'sime3kin.eps','epsc')
figure,
bot.plot(qdyn, 'delay', 5/length(time))

%% Fix eps
!epsfixer.sh






  
