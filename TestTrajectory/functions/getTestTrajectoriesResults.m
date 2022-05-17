function [ fxError, fyError, trajectory] = getTestTrajectoriesResults(varargin)
% getTestTrajectoriesResults
% 10/07/2020
% Get the a table with fiting errors for a range of trajectories defined
% for a desired speed and a radius of curvature.
%
% Usage:
%     getTestTrajectoriesResults(argument_name, argument_value)
%     arguments:
%         R         Radius of the trajectory [m]
%         vx     	Longitudinal velocity of the car for the test [km/h]
%
% Examples:
% [fxError, fyError, trajectory] = getTestTrajectoriesResults('rMax', 1000, 'vx', 20);
%
% Dependencies:
%     getVirtualCarSimulation.m
%     Trajectory.m

    R = [];
    vx = [];
    T = [];
    N = [];
    
    
    for i = 1 :2: nargin
        switch varargin{i}
            case 'T'
                T = varargin{i+1};
            case 'N'
                N = varargin{i+1};
            case 'R'
                R = varargin{i+1};
            case 'vx'
                vx = varargin{i+1};    
            otherwise
        end
    end
          
    %% Get data

    path_G = [];
    path_G = generateArcPath(R);
        
    [virtualCarStates, ~ ] = getVirtualCarSimulation('T', T,...
                                                     'N', N,...
                                                     'vx', vx,...
                                                     'path', path_G,...
                                                     'trajectory', 'circular');
        
    xData = [virtualCarStates(1:end).x];
    yData = [virtualCarStates(1:end).y];
    tData = [virtualCarStates(1:end).t];

    %% Get Polynomial Minimization Trajectory
    % Build polynomial trajectory
    numOrder = 5;
    trajectory{1} = Trajectory('First order');
    trajectory{2} = Trajectory('Second order');
    trajectory{3} = Trajectory('Third order');
    trajectory{4} = Trajectory('Fourth order');
    trajectory{5} = Trajectory('Fifth order');

    % Get coefficients from data trajectory
    for j=1:numOrder
        [pol, err] = trajectory{j}.polyfit(trajectory{j}, xData, yData, tData);
        trajectory{j} = trajectory{j}.setPol(trajectory{j}, pol);

        [xPol, dxPol, ddxPol, yPol, dyPol, ddyPol] = trajectory{j}.getTrajectory(trajectory{j}, tData);

        fxError{j} = err(1);
        fyError{j} = err(2);
    end  

    % Get Results
    varNames = {'SteeringWheel',...
                'fxError1', 'fyError1',...
                'fxError2', 'fyError2',...
                'fxError3', 'fyError3',...
                'fxError4', 'fyError4',...
                'fxError5', 'fyError5'};
    results = table(R, fxError{1}(:), fyError{1}(:),...
                       fxError{2}(:), fyError{2}(:),...
                       fxError{3}(:), fyError{3}(:),...
                       fxError{4}(:), fyError{4}(:),...
                       fxError{5}(:), fyError{5}(:),...
                       'VariableNames', varNames);                      
end

