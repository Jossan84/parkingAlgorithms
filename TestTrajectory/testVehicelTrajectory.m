%testVehicleTrajectory
%10/12/2020

close all;
clear;
more off;
clc;

addpath('classes');
addpath('functions');

% Desired path
xc = 0; % [m]
yc = 30; % [m]
r = 30; % [m]
theta0 = 0; % [rad]

% Reference circular path
th = linspace(0, pi/2, 100);
xC =  flip(0 + r * cos(th));
yC =  flip(r -  r* sin(th));

% Parametrization with the distance for the end point.
% l = 15; % [m]
% dx = l/dt; % [m/s]
% omega = dx / r; % [rad]
% 
% x0 = 0;
% y0 = 0;
% x = xc + r * cos(omega + theta0 - pi/2);
% y = yc + r * sin(omega + theta0 - pi/2);

% Parametrization with the distance along the path.
T = 0.04; % [s]
L = 15; % [m]
n = 100;
tEnd = L/((20/3.6)/1);
% t = 0 : T : tEnd;
t = 0 : T : 4;
vx(1) = 0;
% vx(1) = 20/3.6;
for i = 1 : length(t)
    vx(i+1) = transferFunction(20/3.6, vx(i), 0.90, 0.10);
%     vx(i+1) = 20/3.6;
end

%% Inicializar
% Memorias
x_G   = zeros(1, length(vx));
y_G   = zeros(1, length(vx));
yaw_G = zeros(1, length(vx));
% Posición Inicial
x_G(1)   = 0;
y_G(1)   = 0;
yaw_G(1) = 0;
vy = 0;

for k = 2 : length(vx)
    yawRate = vx(k)/r;
    % Incrementos de posición y yaw
    deltaYaw = yawRate * T;
    X = [vx(k) * T; vy * T];
    % Translación
    R = [cos(deltaYaw/2), -sin(deltaYaw/2); sin(deltaYaw/2), cos(deltaYaw/2)];  
    X = R * X;
    % Rotación 
    R = [cos(yaw_G(k-1)), -sin(yaw_G(k-1)); sin(yaw_G(k-1)), cos(yaw_G(k-1))];
    X = R * X;
    % Actualización de la nueva posición con los incrementos 
    x_G(k) = x_G(k - 1) + X(1, 1);
    y_G(k) = y_G(k - 1) + X(2, 1);
    yaw_G(k) = yaw_G(k - 1) + deltaYaw;        
end

figure(1);
ax = gca;   
title(ax,'Vehicle Trajectory');
hold(ax,'on');
grid(ax,'on');
axis(ax,'equal');
xlabel(ax, 'x[m]');
ylabel(ax, 'y[m]');
plot(ax, xC, yC,'k-');
% plot(ax, x, y,'b.-');
plot(ax, x_G, y_G,'m.-');
hold(ax,'off');


rmpath('classes');
rmpath('functions');
