% global2local
% 11/02/2020

function [PathL] = global2local( PathG, Position )

    % Rotation Matrix
    R_GL = [cos(Position.yaw) , sin(Position.yaw);
            -sin(Position.yaw), cos(Position.yaw)];
        
    d    = [Position.x; Position.y];  

    r    = -R_GL*d;                     % Tranlation
    Rot  = R_GL * [PathG.x; PathG.y];   % Rotation
    
    [n m] = size(Rot);
    Trans(1,:) = r(1) .* ones(1,m);
    Trans(2,:) = r(2) .* ones(1,m);
    
    PathL    = Trans + Rot;    % Transformation Homogeneous
    
end
