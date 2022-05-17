%testParkingMapClass
%11/02/2020

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

% Define Parking Spot
sensorResolution = 0.01;   % Resolution of the sensor [m]
parkingSpotDimensions.corridor = 5;   % Corridor width [m]
parkingSpotDimensions.length   = 2.5;   % Parking spot length [m]
parkingSpotDimensions.width    = 5; % Parking spot width [m]
parkingSpotLocation.x          = 5;   % Parking x local location [m]
parkingSpotLocation.y          = 5;% Parking y local location [m]
parkingSpotLocation.yaw        = 0;   % Parking yaw local location [m]

parkingMap = ParkingMap('width', 15, 'heigth', 15, 'resolution', 10);

parkingMap = parkingMap.setParkingSpace(parkingMap, parkingSpotLocation, parkingSpotDimensions, sensorResolution );


figure;
show(parkingMap.map);
ax = gca;   
title(ax,'Grid Map');
hold(ax,'on');
plot(ax,parkingSpotLocation.x ,parkingSpotLocation.y ,'b*');

rmpath('models');
rmpath('functions');
