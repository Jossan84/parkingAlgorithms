function parkingMap = ParkingMap(varargin)

    % Default map
    width = 20;      % Map width, specified as a scalar in meters
    heigth = 10;     % Map height, specified as a scalar in meters.
    resolution = 10; % Grid resolution, specified as a scalar in cells per meter.

    for i = 1 :2: nargin
        switch varargin{i}
            case 'width'
                width = varargin{i + 1};           
            case 'heigth'
                heigth = varargin{i + 1}; 
            case 'resolution'
                resolution = varargin{i + 1};                
            otherwise
                error('Wrong argument');
        end
    end

    % Create Occupancy map
    map = robotics.BinaryOccupancyGrid(width, heigth, resolution);
    
    parkingMap.map = map;
    
    parkingMap.setParkingSpace = @setParkingSpace;
    
end

function [parkingMap] = setParkingSpace(parkingMap, parkingSpotLocation, parkingSpotDimensions, resolution )

% Inputs:
%     parkingSpotDimensions.corridor % Corridor width [m]
%     parkingSpotDimensions.length   % Parking spot length [m]
%     parkingSpotDimensions.width    % Parking spot width [m]
%     parkingSpotLocation.x          % Parking x local location [m]
%     parkingSpotLocation.y          % Parking y local location [m]
%     parkingSpotLocation.yaw        % Parking yaw local location [m]
    
    map = parkingMap.map;
    
    % Set the grid map
    x = [0:resolution:parkingSpotLocation.x-(parkingSpotDimensions.length/2)]';
    y = ((parkingSpotDimensions.width/2) + parkingSpotLocation.y) * ones(size(x));

    x1 = [(parkingSpotDimensions.length/2) + parkingSpotLocation.x : resolution : map.XWorldLimits(2)]';
    y1 = ((parkingSpotDimensions.width/2) + parkingSpotLocation.y) * ones(size(x1));

    x2 = [parkingSpotLocation.x-(parkingSpotDimensions.length/2) : resolution : parkingSpotLocation.x+(parkingSpotDimensions.length/2)]';
    y2 = (parkingSpotLocation.y - (parkingSpotDimensions.width/2)) * ones(size(x2));

    y3 = [(parkingSpotLocation.y - (parkingSpotDimensions.width/2)) : resolution : ((parkingSpotDimensions.width/2)+parkingSpotLocation.y )]';
    x3 = parkingSpotLocation.x-(parkingSpotDimensions.length/2) * ones(size(y3));

    y4 = y3;
    x4 = parkingSpotLocation.x  + (parkingSpotDimensions.length/2) * ones(size(y4));

    x5 = [0 : resolution : map.XWorldLimits(2)]';
    y5 = parkingSpotDimensions.corridor + (parkingSpotDimensions.width/2)+ parkingSpotLocation.y * ones(size(x5));

    setOccupancy(map, [x   y], ones(size(x)));
    setOccupancy(map, [x1 y1], ones(size(x1)));
    setOccupancy(map, [x2 y2], ones(size(x2)));
    setOccupancy(map, [x3 y3], ones(size(x3)));
    setOccupancy(map, [x4 y4], ones(size(x4)));
    setOccupancy(map, [x5 y5], ones(size(x5)));
   
    parkingMap.map = map;
    
end
