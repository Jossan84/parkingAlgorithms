%testParallelParking
%29/01/2020

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
clear statesMachine;

flag = 0;

%% Initialization
% Car Parameters
l             = 2.7;        % [m][Wheelbase]
steeringRatio = 15.8;       % []
gf    = 0.889; % [m] Distance from nose to front axle
gr    = 0.764; % [m] Distance from rear axle to back
width = 1.826; % [m] Car full width

% Simulation parameters
n  = 400;         % Number of simulation steps
T  = 40e-3;       % [s]
t  = 0:T:T*(n-1); % [s]
vx = 5 / 3.6;     % [m/s]
delta = deg2rad(-500) / steeringRatio; % [rad]
carInit.x = 1;
carInit.y = 6.5;

% Define Parking Spot
sensorResolution               = 0.01;% Resolution of the sensor [m]
parkingSpotDimensions.corridor = 6;   % Corridor width [m]
parkingSpotDimensions.length   = 7;   % Parking spot length [m]
parkingSpotDimensions.width    = 3;   % Parking spot width [m]
parkingSpotLocation.x          = 7;   % Parking x local location [m]
parkingSpotLocation.y          = 3;   % Parking y local location [m]
parkingSpotLocation.yaw        = 0;   % Parking yaw local location [m]

% Define Path % Build Parking Map
mapWidth = 20;  % [m]
mapHeigth = 15; % [m]
mapResolution = 10; % [divisions per meter]
pathResolution = 0.01; % [m]
yLane = carInit.y; % Global X coordinate for path

PathForward.x = [0: pathResolution: mapWidth];
PathForward.y = yLane * ones(size(PathForward.x));

PathBackward.x = [0: pathResolution: mapWidth];
PathBackward.y = parkingSpotLocation.y * ones(size(PathBackward.x));

parkingMap = ParkingMap('width', mapWidth, 'heigth', mapHeigth, 'resolution', mapResolution);
parkingMap = parkingMap.setParkingSpace(parkingMap, parkingSpotLocation, parkingSpotDimensions, sensorResolution );

% Build Car
geometricCar = GeometricModel('T',T,'l',l,'Init',struct('t', 0, 'x', carInit.x, 'y', carInit.y, 'v_x',...
                                     vx, 'v_y', 0, 'yaw', 0, 'yawRate',0));

car = geometricCar;

%% Simulation
lookAheadDistance = 4; % Distance to find point
carStates(1)   = car.getStates(car);
direction = 'Forward'; 
status = 1; % Function status
offset = 0; % [path offset for the maneuver aproach] [m]
distanceToParkSpot = -4.75; 

for k=1:n
    
    % Get the car contour
    carContour(k) = car.getContour(car, gr, gf, width);
    
    % Check if there is a collision with some object
    if 0 ~= sum(getOccupancy(parkingMap.map,carContour(k).Points'));
        collision = 1;
%         disp('Collision');
%         break;
    else
        collision = 0; 
    end
    
    % Get the center park spot point in local coordinates
    localParkingSpot = global2local(parkingSpotLocation, car.getStates(car));
       
    if localParkingSpot(1) < distanceToParkSpot
        parkSpotFound = 1;
    else
        parkSpotFound = 0;
    end
        
    [ direction, pathOffset, status ] = statesMachine( collision, parkSpotFound, offset, status );
    
    state(k) = status;
    switch status
        case 1
            vx = 5 / 3.6;     
            Path_L = global2local(PathForward, car.getStates(car));
            
            Path_L(2,:) = Path_L(2,:) + pathOffset ;
            
            % Get the Path that is Forward
            Path_L  = Path_L(:, Path_L(1, :) > -1e-10);            
            
            % Check if the map ends
            if isempty(Path_L)
                disp('End of Map');
                break;
            end
            
            [goalPoint  ,minDistIndex]  = getLookAheadPoint('IDE', IDE, 'Path_L', Path_L,...
                                                            'pointDistance', lookAheadDistance);
            
        case 2
            vx = -5 / 3.6;
            Path_L = global2local(PathBackward, car.getStates(car));
            
            goalPoint = getPoint( Path_L, lookAheadDistance );
            
        case 3
            vx = 5 / 3.6;
            Path_L = global2local(PathBackward, car.getStates(car));
            
            % Get the Path that is Forward
            Path_L  = Path_L(:, Path_L(1, :) > -1e-10);            
            
            % Check if the map ends
            if isempty(Path_L)
                disp('End of Map');
                break;
            end
           
            [goalPoint  ,minDistIndex]  = getLookAheadPoint('IDE', IDE, 'Path_L', Path_L,...
                                                            'pointDistance', lookAheadDistance);
                                                        
        case 4
            vx = 0;
            
        otherwise
    end        
        
    % Control 
    delta(k)  = atan2(2 * l * goalPoint(2), lookAheadDistance^2); 
    
    car   = car.update(car, delta(k), vx);
    carStates(k+1) = car.getStates(car); 
       
end

%% Plots

plotSimulation;

figure;
plot(t,rad2deg(delta),'b.-');
title('Wheel Angle');
ylabel('delta [deg]');
xlabel('time [s]');

rmpath('models');
rmpath('functions');
