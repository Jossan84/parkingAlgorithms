function [ Path_L] = globalPath2localPath( Path_G, Position )

%     [points] = Path_G.getPathPoints(Path_G);
%         
%     points_G.x = points(1,:);
%     points_G.y = points(2,:);
    points_G.x = Path_G(:,1)';
    points_G.y = Path_G(:,2)';
    
    % Rotation Matrix
    R_GL = [cos(Position.yaw) , sin(Position.yaw);
            -sin(Position.yaw), cos(Position.yaw)];
        
    d    = [Position.x; Position.y];  

    r    = -R_GL*d;                       % Tranlation
    Rot  = R_GL * [points_G.x; points_G.y];   % Rotation
    
    [n m] = size(Rot);
    Trans(1,:) = r(1) .* ones(1,m);
    Trans(2,:) = r(2) .* ones(1,m);
    
    points_L    = Trans + Rot;    % Transformation Homogeneous
    
%     Path_L = Path_G.setPathPoints(Path_G,points_L);
    Path_L = points_L;
    
end
