function [ carStates , delta] = getVirtualCarSimulation( varargin )
% getVirtualCarSimulation
% 10/07/2020
% Get the states of a vhehicle from a simulation.
%
% Usage:
%     getVirtualCarSimulation(argument_name, argument_value)
%     arguments:
%         T       	    Sample time for the simulation
%         steps     	Number of steps for the simulation
%         vx     	    Longitudinal velocity of the car for simulation [m/s]
%         path          Global path
%         trajectory    Type of trajectory {'lane change', 'circular'}
%
% Examples:
%     getVirtualCarSimulation('T', 40e-3, 'steps', 100, 'vx', 5, 'path', path);
%     getVirtualCarSimulation('T', 40e-3, 'steps', 100, 'vx', 5, 'path', path, 'trajectory', 'circular');
%
% Dependencies:
%     GeometricModel.m

     % Default simulation parameters
     T = []; % Number of simulation steps
     N = []; % [s]
     vx = 0 * ones(N,1);  % [m/s]
     trajectory = '';
     delta(1) = 0;
     % Default car parameters
     l = 2.7;                    % [m][Wheelbase]
     steeringRatio = 15.8;
    
     % Default look ahead and path parameters
     iBezier = 1;
     tBezier = 0.0;
     tOffset = 0.3; % To search goalPoint ahead to the car
     maxDistance = 0.1;
     maxIter = 25;
     pointDistance = 15;    
     
          
     for i = 1 :2: nargin
            switch varargin{i}
                case 'T'
                    T = varargin{i+1};
                case 'N'
                    N = varargin{i+1};
                case 'vx'
                    vx = varargin{i+1}; 
                case 'path'
                    path_G = varargin{i+1};
                case 'trajectory'
                    trajectory = varargin{i+1};     
                otherwise
                    error('Wrong argument');
            end
     end
             
%     yawRateInit = 0.270061728395062;
%     yawRateInit = vx(1)*tan(deltaS/steeringRatio)/l;

    % Init Car
    carInit = struct('t', 0, 'x', 0, 'y', 0, 'v_x', 0, 'v_y', 0, 'yaw', 0, 'yawRate', 0);
       
    % Build Car
    geometricCar = GeometricModel('T', T, 'l', l, 'Init', carInit);
    car = geometricCar;

    %% Simulation
    carStates(1) = car.getStates(car);
    for k=1:N
        % Add dynamics to vehicle speed
        vx(k) = transferFunction(vx(k), carStates(k).v_x, 0.9, 0.10);     
        
        if strcmp(trajectory, 'circular')
              path_L = globalPath2localPath(path_G,car.getStates(car));
              
              %Get the Path that is Forward
              path_L  = path_L(:, path_L(1, :) > -1e-10);
              
              [goalPoint, ~] = getLookAheadPoint('IDE', 'Matlab', 'Path_L', path_L, 'pointDistance', pointDistance);      
              delta(k)      = atan2(2 * l * goalPoint(2), pointDistance^2);
        else  
%             % TO DO: Refactoring
              error('Pendent to do refactoring for bezier path implementation');  
%             % From Global to Local Path                                                      
%             path_L = globalPath2localPath(path_G,car.getStates(car));
% 
%             [goalPoint, isFound, nIter1] = path_L.getLookAheadPoint(path_L,0,0,pointDistance,iBezier,...
%                                                               tBezier+tOffset,maxDistance,maxIter,'Newton');
% 
%             delta(k)      = atan2(2 * l * goalPoint(2), pointDistance^2);          
        end
        
        car   = car.update(car, delta(k), vx(k));
        carStates(k+1) = car.getStates(car); 
        
    end
    delta = [delta, delta(end)]; 
end
