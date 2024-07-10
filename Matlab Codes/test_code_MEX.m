%% Définition des paramètres
g = 9.81; % m/s^2
x0 = 0;
y0 = 1.6; % m
z0 = 2.3; % règle fléchette
v0 = 16; % m/s
theta = 3; % degrès
phi = 1; % degrès
deltat = 1/120; % Fréquence des caméras
t = 0:deltat:0.15;

F = 0;
m = 23e-3;

%% Modélisation
x = x0 + v0*t(1:13)*cos(theta*2*pi()/360)*sin(phi*2*pi()/360) 
y = v0*t(1:13)*sin(theta*2*pi()/360)-(1/2)*g*(t(1:13).^2)+y0
z = z0 - v0*t(1:13)*cos(theta*2*pi()/360)*cos(phi*2*pi()/360)

pred_traj3D(z,x,y)