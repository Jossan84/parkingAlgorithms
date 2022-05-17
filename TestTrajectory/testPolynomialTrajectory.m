%testPolynomialTrajectory
%07/07/2020

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

addpath('classes');
addpath('functions');
addpath('results');

%% Get data from simulation
T = 40e-3;
N = 100;

% Desired speed
u = 20/3.6 * ones(N,1); % [m/s] 

% Desired path
R = 30; 
[path_G] = generateArcPath(R);
% path_G = Path('Bezier3');

[virtualCarStates, ~ ] = getVirtualCarSimulation('T', T,...
                                                 'N', N,...
                                                 'vx', u,...
                                                 'path', path_G,...
                                                 'trajectory','circular');                                       
                                       
xData = [virtualCarStates(1:end).x];
yData = [virtualCarStates(1:end).y];
tData = [virtualCarStates(1:end).t];


%% Get Polynomial Minimization Trajectory
% Build polynomial trajectory
polynomialStates = struct;

trajectory = Trajectory('Third order');
% % trajectory = Trajectory('Second order');
% trajectory = Trajectory('Fifth order');

[pol, err] = trajectory.polyfit(trajectory, xData, yData, tData);
trajectory = trajectory.setPol(trajectory, pol);

% [xPol, dxPol, ddxPol, yPol, dyPol, ddyPol] = trajectory.getTrajectory(trajectory, tData);
[polynomialStates.x, polynomialStates.vx, polynomialStates.ax,...
 polynomialStates.y, polynomialStates.vy, polynomialStates.ay] = trajectory.getTrajectory(trajectory, tData);
 
%% Plots
plotTrajectory( virtualCarStates, polynomialStates );

figure(2);
plot(xData, yData, 'b.-');
hold on;
plot(path_G(:, 1), path_G(:, 2), 'k.-')
axis equal;
grid on;
plot(polynomialStates.x, polynomialStates.y, 'r.-');
xlabel('x [m]');
ylabel('y [m]');

polynomialAverageVelocity = mean(sqrt(polynomialStates.vx.^2 + polynomialStates.vy.^2))
err

rmpath('classes');
rmpath('functions');
rmpath('results');

