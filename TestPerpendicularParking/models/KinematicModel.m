function kinematicModel = KinematicModel(varargin)
    
    % Default params
    params.T             = 40e-3;% [s]
    params.l             = 5;% [m]
    params.l_f           = 0.4 * params.l;% [m]
    params.l_r           = 0.6 * params.l;% [m]
    
   % Defaul states
    states = struct('t', 0, 'x', 0, 'y', 0, 'v_x', 0, 'v_y', 0, 'yaw', 0, 'yawRate',0);
    
    for i = 1 :2: nargin
        switch varargin{i}
            case 'T'
                params.T = varargin{i + 1};           
            case 'l'
                params.l = varargin{i + 1}; 
            case 'Init'
                states = varargin{i + 1};                
            otherwise
                error('Wrong argument');
        end
    end
    
    % Continuous System
    system.A = [0,0;
                0,0];
    system.B = [(states.v_x * params.l_r)/params.l;
                states.v_x/params.l]; 
    system.C = eye(2);
    system.D = 0;
    
    % Discrete System
    system.Ad = system.A;
    system.Bd = system.B; 
    system.Cd = system.C;
    system.Dd = system.D;
    
    kinematicModel.params = params;
    kinematicModel.states = states;
    kinematicModel.system = system;
    
    kinematicModel.update    = @update;
    kinematicModel.getStates = @getStates;
    kinematicModel.setStates = @setStates;
    kinematicModel.getSystem = @getSystem;

end

function [kinematicModel] = update(kinematicModel,delta,v_x)
    
% %% Get current global coordinates and yaw [rear axe]  
%     X = [v_x * kinematicModel.params.T; kinematicModel.states.v_y * kinematicModel.params.T];
%     
%     R = [cos(kinematicModel.states.yaw), -sin(kinematicModel.states.yaw);...
%          sin(kinematicModel.states.yaw), cos(kinematicModel.states.yaw)];
%     X = R * X;
%     % Translate from rear axle to center of vehicle
%     X_center = [kinematicModel.states.x; kinematicModel.states.y] + R * [kinematicModel.params.l_r; 0]; 
%     X_center = X_center + X;
% 
%     kinematicModel.states.yaw = kinematicModel.states.yaw + kinematicModel.params.T * kinematicModel.states.yawRate;
%     R   = [cos(kinematicModel.states.yaw), -sin(kinematicModel.states.yaw);
%            sin(kinematicModel.states.yaw), cos(kinematicModel.states.yaw)];
%     % Translate from center to rear axle   
%     X = X_center + R * [-kinematicModel.params.l_r; 0];     
%     kinematicModel.states.x = X(1, 1);
%     kinematicModel.states.y = X(2, 1);
%         
% %% Compute system
%     X = kinematicModel.system.Ad * X + kinematicModel.system.Bd * tan(delta);
%     
% %% Update next states
%     kinematicModel.states.v_x     = v_x;
%     kinematicModel.states.v_y     = X(1);
%     kinematicModel.states.yawRate = X(2);
%     kinematicModel.states.t       = kinematicModel.states.t + kinematicModel.params.T;
% %%

     kinematicModel.states.yawRate = v_x / kinematicModel.params.l * tan(delta);
%      X = v_x * kinematicModel.params.T * [cos(kinematicModel.params.T * kinematicModel.states.yawRate / 2);...
%                                           sin(kinematicModel.params.T * kinematicModel.states.yawRate / 2)];
    X = v_x * kinematicModel.params.T * [1;...
                                         0];
    
    % Rotation matrix
    R = [cos(kinematicModel.states.yaw), -sin(kinematicModel.states.yaw);
         sin(kinematicModel.states.yaw), cos(kinematicModel.states.yaw)];
    X = R * X;
    dx = X(1,1);
    dy = X(2,1);
    dyaw = kinematicModel.params.T * kinematicModel.states.yawRate;
    
    kinematicModel.states.x   = kinematicModel.states.x    + dx;
    kinematicModel.states.y   = kinematicModel.states.y    + dy;
    kinematicModel.states.yaw = kinematicModel.states.yaw  + dyaw;
    
    kinematicModel.states.v_x = v_x;
    % Change if the point is moved from rear axe to other point of the car
    kinematicModel.states.v_y = (v_x * kinematicModel.params.l_r) / kinematicModel.params.l * tan(delta);
    %kinematicModel.states.v_y = 0;
    kinematicModel.states.t = kinematicModel.states.t + kinematicModel.params.T;
    
    
end

function states = getStates(kinematicModel)
    states = kinematicModel.states;
end

function kinematicModel = setStates(kinematicModel, states)
    kinematicModel.states = states;
end

function system = getSystem(kinematicModel)
    system = kinematicModel.system;
end
