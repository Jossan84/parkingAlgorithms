%testPerpendicularParking
%18/02/2020

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

prompt = {'Options: [0]Path Following [1]Parking Go to one Point'};
dlgtitle = 'Choose option';
definput = {'0'};
opts.Interpreter = 'tex';
answer = inputdlg(prompt,dlgtitle,[1 60],definput,opts);

if (str2num(answer{1}) == 0)
    goOnePoint = 0;
else
    goOnePoint = 1;
end

flag = 0;

%% Initialization
% Car Parameters
l             = 2.7;        % [m][Wheelbase]
steeringRatio = 15.8;       % []
gf    = 0.889; % [m] Distance from nose to front axle
gr    = 0.764; % [m] Distance from rear axle to back
width = 1.826; % [m] Car full width

% Simulation parameters
n  = 300;         % Number of simulation steps
T  = 40e-3;       % [s]
t  = 0:T:T*(n-1); % [s]
vx = 5 / 3.6;     % [m/s]
delta = deg2rad(-500) / steeringRatio; % [rad]

% Define Parking Spot
sensorResolution = 0.01;   % Resolution of the sensor [m]
parkingSpotDimensions.corridor = 6;   % Corridor width [m]
parkingSpotDimensions.length   = 3;   % Parking spot length [m]
parkingSpotDimensions.width    = 6;   % Parking spot width [m]
parkingSpotLocation.x          = 5;   % Parking x local location [m]
parkingSpotLocation.y          = 5;   % Parking y local location [m]
parkingSpotLocation.yaw        = 0;   % Parking yaw local location [m]

% Define Path % Build Parking Map
mapWidth = 20;  % [m]
mapHeigth = 15; % [m]
mapResolution = 10; % [divisions per meter]
pathResolution = 0.01; % [m]
xLane = 10; % Global X coordinate for path

PathForward.x = [0: pathResolution: mapWidth];
PathForward.y = xLane * ones(size(PathForward.x));

PathBackward.y = [0: pathResolution: mapWidth];
PathBackward.x = parkingSpotLocation.x * ones(size(PathBackward.y));

parkingMap = ParkingMap('width', mapWidth, 'heigth', mapHeigth, 'resolution', mapResolution);
parkingMap = parkingMap.setParkingSpace(parkingMap, parkingSpotLocation, parkingSpotDimensions, sensorResolution );

% Build Car
geometricCar = GeometricModel('T',T,'l',l,'Init',struct('t', 0, 'x', 1, 'y', 10, 'v_x',...
                                     vx, 'v_y', 0, 'yaw', 0, 'yawRate',0));

car = geometricCar;

%% Simulation
lookAheadDistance = 4; % Distance to find point
carStates(1)   = car.getStates(car);
direction = 'Forward'; 
status = 1; % Function status
offset = 3; % [path offset for the maneuver aproach] [m]
distanceToParkSpot = 0.5; 

for k=1:n
    
    % Get the car contour
    carContour(k) = car.getContour(car, gr, gf, width);
    
    % Check if there is a collision with some object
    if 0 ~= sum(getOccupancy(parkingMap.map,carContour(k).Points'));
        collision = 1;
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
    
    % Path following and aproach maneuver
    if ( strcmp(direction, 'Forward') && (status == 1) )
        
        vx = 5 / 3.6;     % [m/s]
        Path_L = global2local(PathForward, car.getStates(car));
       
        Path_L(2,:) = Path_L(2,:) + pathOffset ;     
        
        % Get the Path that is Forward
        Path_L  = Path_L(:, Path_L(1, :) > -1e-10);
        
        % Check if the map ends
        if isempty(Path_L)
            disp('End of Map');
            break;
        end
        
        %goalPoint = getPoint( Path_L, lookAheadDistance ); 
        [goalPoint  ,minDistIndex]  = getLookAheadPoint('IDE', IDE, 'Path_L', Path_L,...
                                                          'pointDistance', lookAheadDistance);
    % Parking maneuver    
    else
                    
        Path_L = global2local(PathBackward, car.getStates(car));
        
        if strcmp(direction, 'Backward')
            vx = -5 / 3.6;     % [m/s]          
        else
            vx = 5 / 3.6;     % [m/s]
            % Get the Path that is Forward
            Path_L  = Path_L(:, Path_L(1, :) > -1e-10);
        end
                     
            if goOnePoint == 1 && flag == 0
                goalPoint =   localParkingSpot;
                deltaP = atan2(2 * l * goalPoint(2), norm(goalPoint)^2);
                goOnePoint = 1;
                flag = 1;
            else    
                goalPoint = getPoint( Path_L, lookAheadDistance );
            end            
    end    
    
    % Control
    if flag == 1
       delta(k)  = deltaP;
    else
       delta(k)  = atan2(2 * l * goalPoint(2), lookAheadDistance^2); 
    end
    
    car   = car.update(car, delta(k), vx);
    carStates(k+1) = car.getStates(car); 
       
end

%% Plots
% figure;
% show(parkingMap.map);
% ax = gca;   
% title(ax,'Grid Map');
% hold(ax,'on');
% plot(ax,parkingSpotLocation.x ,parkingSpotLocation.y ,'b*');
% plotVehiclePose(ax, [carStates(1).x], [carStates(1).y], [carStates(1).yaw]);
% plot(ax, [carStates(2:end-1).x], [carStates(2:end-1).y],'r.-');
% plotVehiclePose(ax, [carStates(end).x], [carStates(end).y], [carStates(end).yaw]);
% plot(ax, [PathForward.x], [PathForward.y],'--k');
% plot(ax, [PathBackward.x], [PathBackward.y],'--k');
% plot(ax,carContour.Points(1,:),carContour.Points(2,:),'b.-');

plotSimulation;

figure;
plot(t,rad2deg(delta),'b.-');
title('Wheel Angle');
ylabel('delta [deg]');
xlabel('time [s]');

rmpath('models');
rmpath('functions');
