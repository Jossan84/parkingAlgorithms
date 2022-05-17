%testGetContour
%17/02/2020

close all;
clear;
more off;
clc;

if exist('OCTAVE_VERSION', 'builtin') ~= 0% OCTAVE
    IDE = 'OCTAVE';
    markerSize = 12;
else% MATLAB
    IDE = 'MATLAB';
    markerSize = 6;
end

addpath('models');
addpath('functions');

% Car Parameters
l             = 2.7;        % [m][Wheelbase]
steeringRatio = 15.8;       % []

% Simulation parameters
n  = 1;         % Number of simulation steps
T  = 40e-3;       % [s]
t  = 0:T:T*(n-1); % [s]
vx = 5 / 3.6;     % [m/s]
delta = deg2rad(-200) / steeringRatio; % [rad]

% Build Car
geometricCar = GeometricModel('T',T,'l',l,'Init',struct('t', 0, 'x', 0, 'y', 0, 'v_x',...
                                     vx, 'v_y', 0, 'yaw', pi/4, 'yawRate',0));

car = geometricCar;
carStates(1)   = car.getStates(car);

for k=1:n  
    %car   = car.update(car, delta, vx);
    %carStates(k+1) = car.getStates(car); 
end

%% Plots
figure;
ax = gca;
title(ax,'Plot Vehicle');
hold(ax,'on');
plot(ax, [carStates(1).x], [carStates(1).y],'b.');
plotVehiclePose(ax, [carStates(1).x], [carStates(1).y], [carStates(1).yaw]);
ylim(ax,[-5 5]);
xlim(ax,[-5 5]);

carContour = car.getContour(car, 1, 1, 2.5);
plot(ax,carContour.Points(1,:),carContour.Points(2,:),'b.-');

rmpath('models');
rmpath('functions');
