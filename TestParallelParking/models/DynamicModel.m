function dynamicModel = DynamicModel(varargin)
    
    % Default params
    params.T             = 40e-3;% [s]
    params.m             = 1500;% [kg]
    params.I_z           = 1000;% [N m s^2]
    params.C_f           = 20000;% [N/rad]
    params.C_r           = 30000;% [N/rad]
    params.l             = 5;% [m]
    params.l_f           = 0.4 * params.l;% [m]
    params.l_r           = 0.6 * params.l;% [m]
 
    % Default states
    states = struct('t', 0, 'x', 0, 'y', 0, 'v_x', 0, 'v_y', 0, 'yaw', 0, 'yawRate',0);
    
    for i = 1 :2: nargin
        switch varargin{i}
            case 'T'
                params.T = varargin{i + 1};
            case 'm'
                params.m = varargin{i + 1};
            case 'I_z'
                params.I_z = varargin{i + 1};
            case 'C_f'
                params.C_f = varargin{i + 1};
            case 'C_r'
                params.C_r = varargin{i + 1};               
            case 'l'
                params.l = varargin{i + 1};
            case 'Init'
                states = varargin{i + 1};                  
            otherwise
                error('Wrong argument');
        end
    end
    
    params.l_f           = 0.4 * params.l;% [m]
    params.l_r           = 0.6 * params.l;% [m]
    
    %Continuous System
    system.A = [-(params.C_f + params.C_r) / params.m,...
                (-params.C_f * params.l_f + params.C_r * params.l_r) / params.m;
                (-params.C_f * params.l_f + params.C_r * params.l_r) / params.I_z,...
                -(params.C_f * params.l_f^2 + params.C_r * params.l_r^2) / params.I_z];
    system.B = [params.C_f / params.m;
                params.C_f * params.l_f / params.I_z]; 
    system.C = eye(2);
    system.D = 0;
    
    %Discrete System
    A_k = system.A * (1 / states.v_x) + [0, -states.v_x; 0, 0];
    system.Ad = expm(A_k * params.T);
    system.Bd = (system.Ad - eye(2)) * inv(A_k) * system.B;
    system.Cd = system.C;
    system.Dd = system.D;
    
    dynamicModel.params = params;
    dynamicModel.system = system;
    dynamicModel.states = states;
    
    dynamicModel.update    = @update;
    dynamicModel.getStates = @getStates;
    dynamicModel.setStates = @setStates;
    dynamicModel.getSystem = @getSystem;

end

function [dynamicModel] = update(dynamicModel,delta,v_x)
   
%% Get current global coordinates and yaw [rear axe]
    X = [v_x * dynamicModel.params.T; dynamicModel.states.v_y * dynamicModel.params.T];
    R = [cos(dynamicModel.states.yaw), -sin(dynamicModel.states.yaw);...
         sin(dynamicModel.states.yaw), cos(dynamicModel.states.yaw)];
    X = R * X;
    % Translate from rear axle to center of vehicle
    X_center = [dynamicModel.states.x; dynamicModel.states.y] + R * [dynamicModel.params.l_r; 0]; 
    X_center = X_center + X;

    dynamicModel.states.yaw = dynamicModel.states.yaw + dynamicModel.params.T * dynamicModel.states.yawRate;
    R   = [cos(dynamicModel.states.yaw), -sin(dynamicModel.states.yaw);
           sin(dynamicModel.states.yaw), cos(dynamicModel.states.yaw)];
    % Translate from center to rear axle
    X = X_center + R * [-dynamicModel.params.l_r; 0];     
    dynamicModel.states.x = X(1, 1);
    dynamicModel.states.y = X(2, 1);
    
%% Compute system
    A_k = dynamicModel.system.A * (1 / v_x) + [0, -v_x; 0, 0];
    X   = [dynamicModel.states.v_y;
           dynamicModel.states.yawRate];
     
    %Discretize system
    dynamicModel.system.Ad = expm(A_k * dynamicModel.params.T);
    dynamicModel.system.Bd = (dynamicModel.system.Ad - eye(2)) * inv(A_k) * dynamicModel.system.B;
    
    X = dynamicModel.system.Ad * X + dynamicModel.system.Bd * delta;
    
%% Update next states
    dynamicModel.states.v_x     = v_x;
    dynamicModel.states.v_y     = X(1);
    dynamicModel.states.yawRate = X(2);
    
    dynamicModel.states.t       = dynamicModel.states.t + dynamicModel.params.T;
%%
end

function states = getStates(dynamicModel)
    states = dynamicModel.states;
end

function dynamicModel = setStates(dynamicModel, states)
    dynamicModel.states = states;
end

function system = getSystem(dynamicModel)
    system = dynamicModel.system;
end
