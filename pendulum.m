function dx = pendulum(t,x)
global dx

x1 = x(1);
x2 = x(2);
a = 1; b = 1; c = 1; K1 = 1; K2 = 1;

%%% Control input
% u = 0;

%%% Lyapunov redesign
% disturbance
d = 0.5*(1+0.2*sin(2*t));
% d = 0;
% v = 0;
% form 1:
G = c;
w = G*[x1 x2];
n = [0.6 0.6];
K = 0.1;
% v = -(1/(1-K))*n*w';
% form 2:
% v = -(1/(1-K))*n*tanh(100*w)';
% form 3:
nw = norm(w,2);
E = 0.01;
if(nw < E)
    v = -(1/(1-K))*n*(w'/E);
else
    v = -(1/(1-K))*n*(w'/nw);
end
% overall control
u = a*(sin(x1))-(K1*x1+K2*x2)+v;

%%% Stabilizer for backstepping
% u = -a*sin(x1)+K1*x1;

dx(1,1) = x2;
dx(2,1) = -a*sin(x1)-b*x2+c*(u+d);