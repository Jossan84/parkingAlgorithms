%testPolynomialCurvature
%13/07/2020

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

%% Simulation 
% Simulation parameters
T = 40e-3;
N = 100;

% Car parameters
steeringRatio = 15.8;
l = 2.7; 

% Desired speed
u = 20/3.6 * ones(N,1); % [m/s] 

% Desired path
R = 30; 
[path_G] = generateArcPath(R);
% path_G = Path('Bezier3');


% Compute trajectory
[virtualCarStatesStationary, deltaCarStationary ] = getVirtualCarSimulation('T', T,...
                                                        'N', N,...
                                                        'vx', u,...
                                                        'path', path_G,...
                                                        'trajectory','circular'); 
                                                    
xData = [virtualCarStatesStationary(1:end).x];
yData = [virtualCarStatesStationary(1:end).y];
tData = [virtualCarStatesStationary(1:end).t];
deltaData = rad2deg([deltaCarStationary .* steeringRatio]);

rData = [virtualCarStatesStationary.v_x] ./ [virtualCarStatesStationary.yawRate];
rData(1) = R;
kData = 1 ./ rData;


%% Get Polynomial Curvature Trajectory 
polynomialOrder = 'Second order';
% polynomialOrder = 'Third order';
polynomialTrajectoryStates = struct;

% Build polynomial trajectory
polynomialTrajectory = Trajectory(polynomialOrder);

[trajectoryParameters, trajectoryCost] = polynomialTrajectory.polyfit(polynomialTrajectory, xData, yData, tData);
polynomialTrajectory = polynomialTrajectory.setPol(polynomialTrajectory, trajectoryParameters);

% Get states from the polynomic
[polynomialTrajectoryStates.x, polynomialTrajectoryStates.vx, polynomialTrajectoryStates.ax,...
 polynomialTrajectoryStates.y, polynomialTrajectoryStates.vy, polynomialTrajectoryStates.ay] = polynomialTrajectory.getTrajectory(polynomialTrajectory, tData);

% Get the curvature
kPolynomialTrajectory = getCurvature( polynomialTrajectoryStates.vx, polynomialTrajectoryStates.vy,...
                                      polynomialTrajectoryStates.ax, polynomialTrajectoryStates.ay );

%% Get Polynomial Curvature Coefficients
polynomialCurvature = Polynomial(polynomialOrder);

[polynomilaCurvatureParameters, curvatureCost] = polynomialCurvature.polyfit(polynomialCurvature, kData, tData);
polynomialCurvature = polynomialCurvature.setPol(polynomialCurvature, polynomilaCurvatureParameters);

[ kPolynomialCurvature, dkPolynomialCurvature, ddkPolynomialCurvature] = polynomialCurvature.getPolynomic(polynomialCurvature, tData);

%% Get Turning Radius
rPolynomialTrajectory = 1 ./ kPolynomialTrajectory;
rPolynomialCurvature = 1 ./ kPolynomialCurvature;

%% Get Steering Wheel Angle
deltaSPolynomialTrajectory = rad2deg(atan(l./rPolynomialTrajectory) * steeringRatio);
deltaSPolynomialCurvature = rad2deg(atan(l./rPolynomialCurvature) * steeringRatio);

%% Plots
plotTrajectory( virtualCarStatesStationary, polynomialTrajectoryStates );

plotCurvature( tData,...
               kData, kPolynomialTrajectory, kPolynomialCurvature,...
               rData, rPolynomialTrajectory, rPolynomialCurvature,...
               deltaData, deltaSPolynomialTrajectory, deltaSPolynomialCurvature);

rmpath('classes');
rmpath('functions');
rmpath('results');

