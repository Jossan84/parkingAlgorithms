function [x, y] = circle(xCenter, yCenter, r)
    th = 0:pi/50:2*pi;
    x = r * cos(th) + xCenter;
    y = r * sin(th) + yCenter;
end

