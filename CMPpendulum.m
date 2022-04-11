clear all
close all
clc

global dx
dx = zeros(2,1);
xp = zeros(3,1);

[t,x] = ode45('pendulum',[0 20],[-pi/2 0]);
x1 = x(:,1);
x2 = x(:,2);
x1d = zeros(size(t));
plot(t,x1,'k','LineWidth',2)
hold on
plot(t,x1d,'k--','LineWidth',2)
title('Rediseño de Lyapunov forma 3','FontSize',14)
xlabel('Tiempo [s]','FontSize',14)
ylabel('Posición angular [rad]','FontSize',14)
legend('x1','equilibrio')
grid

% figure(2)
% d = 0.5*(1+0.2*sin(2*t));
% plot(t,d,'k','LineWidth',2)
% title('Señal de perturbación','FontSize',14)
% xlabel('Tiempo [s]','FontSize',14)
% ylabel('Par [Nm]','FontSize',14)
% grid

% [t,xb] = ode45('pendulum_backstepping',[0 20],[-pi/2 0 0]);
% xb1 = xb(:,1);
% xb2 = xb(:,2);
% xb3 = xb(:,2);
% figure(2)
% plot(t,xb1,'LineWidth',2)
% title('Backstepping')
% grid