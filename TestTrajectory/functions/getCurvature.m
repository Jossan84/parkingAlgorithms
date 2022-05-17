function [ k ] = getCurvature( dx, dy, ddx, ddy )

    k = (dx.*ddy - dy.*ddx)./ ((dx.^2 + dy.^2).^(3/2));
    
end

