function geometricModel = GeometricModel(varargin)
    
    % Default params
    params.T             = 40e-3;% [s]
    params.l             = 5;% [m]
%     params.l_f           = 0.4 * params.l;% [m]
%     params.l_r           = 0.6 * params.l;% [m]
    
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
    system.B = [0,1];
    system.C = eye(2);
    system.D = 0;
    
    % Discrete System
    system.Ad = system.A;
    system.Bd = system.B; 
    system.Cd = system.C;
    system.Dd = system.D;
    
    geometricModel.params = params;
    geometricModel.states = states;
    geometricModel.system = system;
    
    geometricModel.update    = @update;
    geometricModel.getStates = @getStates;
    geometricModel.setStates = @setStates;
    geometricModel.getSystem = @getSystem;
    geometricModel.setSystem = @setSystem;
    geometricModel.getContour = @getContour;

end

function [geometricModel] = update(geometricModel, delta, v_x)
    
% %% Get current global coordinates and yaw [rear axe]  
%     X = [v_x * geometricModel.params.T; geometricModel.states.v_y * geometricModel.params.T];
%     
%     R = [cos(geometricModel.states.yaw), -sin(geometricModel.states.yaw);...
%          sin(geometricModel.states.yaw), cos(geometricModel.states.yaw)];
%     X = R * X;
%     % Translate from rear axle to center of vehicle
%     X_center = [geometricModel.states.x; geometricModel.states.y] + R * [geometricModel.params.l_r; 0]; 
%     X_center = X_center + X;
% 
%     geometricModel.states.yaw = geometricModel.states.yaw + geometricModel.params.T * geometricModel.states.yawRate;
%     R   = [cos(geometricModel.states.yaw), -sin(geometricModel.states.yaw);
%            sin(geometricModel.states.yaw), cos(geometricModel.states.yaw)];
%     % Translate from center to rear axle   
%     X = X_center + R * [-geometricModel.params.l_r; 0];     
%     geometricModel.states.x = X(1, 1);
%     geometricModel.states.y = X(2, 1);
%  

%% Compute system
    X = [geometricModel.states.v_y;geometricModel.states.yawRate];
%     u = [v_x * geometricModel.params.l_r /geometricModel.params.l;
%          v_x / geometricModel.params.l] * tan(delta);
    u = (v_x / geometricModel.params.l) * tan(delta);
                 
    X = geometricModel.system.Ad * X + geometricModel.system.Bd' * u;
    
%% Update next states
    geometricModel.states.v_x     = v_x;
    geometricModel.states.v_y     = X(1);
    geometricModel.states.yawRate = X(2);
    geometricModel.states.t       = geometricModel.states.t + geometricModel.params.T;
    
%% Update position
    dyaw = geometricModel.params.T * geometricModel.states.yawRate;
    
    X = [v_x * geometricModel.params.T; geometricModel.states.v_y * geometricModel.params.T];
    R = [cos(dyaw/2), -sin(dyaw/2); sin(dyaw/2), cos(dyaw/2)];
    X = R * X;
    
    R = [cos(geometricModel.states.yaw), -sin(geometricModel.states.yaw);...
         sin(geometricModel.states.yaw), cos(geometricModel.states.yaw)];
    X = R * X; 
    dx = X(1,1);
    dy = X(2,1);
    
    geometricModel.states.x   = geometricModel.states.x    + dx;
    geometricModel.states.y   = geometricModel.states.y    + dy;
    geometricModel.states.yaw = geometricModel.states.yaw  + dyaw;
    
    geometricModel.states.v_y = 0;
end

function states = getStates(geometricModel)
    states = geometricModel.states;
end

function geometricModel = setStates(geometricModel, states)
    geometricModel.states = states;
end

function system = getSystem(geometricModel)
    system = geometricModel.system;
end

function geometricModel = setSystem(geometricModel, A, B)
    % Continuous System
    geometricModel.system.A = A;
    geometricModel.system.B = B;
    geometricModel.system.C = eye(2);
    geometricModel.system.D = 0;
    
    % Discrete System
    geometricModel.system.Ad = geometricModel.system.A;
    geometricModel.system.Bd = geometricModel.system.B; 
    geometricModel.system.Cd = geometricModel.system.C;
    geometricModel.system.Dd = geometricModel.system.D;
end

function contour = getContour(geometricModel, gr, gf, width)
    
    resolution = 0.01;  % [m]
    
    % Rotation matrix
    R = [cos(geometricModel.states.yaw), -sin(geometricModel.states.yaw);...
         sin(geometricModel.states.yaw), cos(geometricModel.states.yaw)];
    
%     %% Get corner points
%     % Corner A 
%     X = [geometricModel.states.x; geometricModel.states.y] + R * [geometricModel.params.l + gf; width/2]; 
%     A = [X(1); X(2)];  
%                 
%     % Corner B     
%     X = [geometricModel.states.x; geometricModel.states.y] + R * [geometricModel.params.l + gf; -width/2];        
%     B = [X(1); X(2)];
% 
%     % Corner C
%     X = [geometricModel.states.x; geometricModel.states.y] + R * [-gr; width/2];
%     C = [X(1); X(2)];
%    
%     % Corner D
%     X = [geometricModel.states.x; geometricModel.states.y] + R * [-gr; -width/2];    
%     D = [X(1); X(2)];

    %% Get contour
    % Segment AB
    x = -gr: resolution: (geometricModel.params.l + gf);
    y = (width/2) * ones(size(x));
    X = [x;y];
    
    AB = [geometricModel.states.x * ones(size(x)); geometricModel.states.y * ones(size(x))] + ...
          R * X;
    
    % Segment BC                
    x = -gr: resolution: (geometricModel.params.l + gf);
    y = (-width/2) * ones(size(x));
    X = [x;y];
    
    BC = [geometricModel.states.x * ones(size(x)); geometricModel.states.y * ones(size(x))] + ...
          R * X;
    
    % Segment CD
    y = -(width/2): resolution: (width/2);
    x = (geometricModel.params.l + gf) * ones(size(y)); 
    X = [x;y];
    
    CD = [geometricModel.states.x * ones(size(x)); geometricModel.states.y * ones(size(x))] + ...
          R * X;   
    
    % Segment DA
    y = -(width/2): resolution: (width/2);
    x = -gr * ones(size(y)); 
    X = [x;y];
    
    DA = [geometricModel.states.x * ones(size(x)); geometricModel.states.y * ones(size(x))] + ...
          R * X;                  
    
%     contour.Points = [A C D B];
    contour.Points = [AB BC CD DA];
    
                    
end
