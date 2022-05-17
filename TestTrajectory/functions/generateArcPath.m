function [path_G] = generateArcPath(R)
    
    yaw = linspace(0, pi/2, 200);
    x = 0 + R * cos(yaw);
    y =  R -  R * sin(yaw);
    
    path_G(:,1) = flip(x);
    path_G(:,2) = flip(y);
end

