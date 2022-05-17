function [trajectory] = Trajectory(varargin)
            
    for i = 1 :2: nargin
        switch varargin{i}
            case 'First order'
                degree = '1';
                a = [0 0];
                b = [0 0];
                
                fx  = @(t) a(1)*t + a(2);
                dfx = @(t)a(1);
                ddfx = @(t)0;
                
                fy  = @(t) b(1)*t + b(2);
                dfy = @(t)b(1);
                ddfy = @(t)0;
                
            case 'Second order'
                degree = '2';
                a = [0 0 0];
                b = [0 0 0];
                
                fx  = @(t) a(1)*t^2 + a(2)*t + a(3);
                dfx = @(t) 2*a(1)*t + a(2);
                ddfx = @(t) 2*a(1);
                
                fy  = @(t) b(1)*t^2 + b(2)*t + b(3);
                dfy = @(t) 2*b(1)*t + b(2);
                ddfy = @(t) 2*b(1);
                             
            case 'Third order'
                degree = '3';
                a = [0 0 0 0];
                b = [0 0 0 0];
                
                fx  = @(t) a(1)*t^3 + a(2)*t^2 + a(3)*t + a(4);
                dfx = @(t) 3*a(1)*t^2 + 2*a(2)*t + a(3);
                ddfx = @(t) 6*a(1)*t + 2*a(2);
                 
                fy  = @(t) b(1)*t^3 + b(2)*t^2 + b(3)*t + b(4);
                dfy = @(t) 3*b(1)*t^2 + 2*b(2)*t + b(3);
                ddfy = @(t) 6*b(1)*t + 2*b(2);
                
            case 'Fourth order'
                degree = '4';
                a = [0 0 0 0 0];
                b = [0 0 0 0 0];
                
                fx  = @(t) a(1)*t^4 + a(2)*t^3 + a(3)*t^2 + a(4)*t + a(5);
                dfx = @(t) 4*a(1)*t^3 + 3*a(2)*t^2 + 2*a(3)*t + a(4);
                ddfx = @(t) 12*a(1)*t^2 + 6*a(2)*t + 2*a(3);
                
                fy  = @(t) b(1)*t^4 + b(2)*t^3 + b(3)*t^2 + b(4)*t + b(5);
                dfy = @(t) 4*b(1)*t^3 + 3*b(2)*t^2 + 2*b(3)*t + b(4);
                ddfy = @(t) 12*b(1)*t^2 + 6*b(2)*t + 2*b(3);
      
            case 'Fifth order'
                degree = '5';
                a = [0 0 0 0 0 0];
                b = [0 0 0 0 0 0];
                
                fx  = @(t) a(1)*t^5 + a(2)*t^4 + a(3)*t^3 + a(4)*t^2 + a(5)*t + a(6);
                dfx = @(t) 5*a(1)*t^4 + 4*a(2)*t^3 + 3*a(3)*t^2 + 2*a(4)*t + a(5);
                ddfx = @(t) 20*a(1)*t^3 + 12*a(2)*t^2 + 6*a(3)*t + 2*a(4);
                
                fy  = @(t) b(1)*t^5 + b(2)*t^4 + b(3)*t^3 + b(4)*t^2 + b(5)*t + b(6);
                dfy = @(t) 5*b(1)*t^4 + 4*b(2)*t^3 + 3*b(3)*t^2 + 2*b(4)*t + b(5);
                ddfy = @(t) 20*b(1)*t^3 + 12*b(2)*t^2 + 6*b(3)*t + 2*b(4);
                
            otherwise
                error('Wrong argument');
        end
    end
    
    pol = [a; b];
    
    trajectory.pol = pol ;
    trajectory.degree = degree;
    trajectory.fx = fx;
    trajectory.dfx = dfx;
    trajectory.ddfx = ddfx;
    trajectory.fy = fy;
    trajectory.dfy = dfy;
    trajectory.ddfy = ddfy;
    
    trajectory.getPoint = @getPoint;
    trajectory.getTrajectory = @getTrajectory;
    trajectory.setPol = @setPol;
    trajectory.polyfit = @polyfit;
    
end

function [x, dx, ddx, y, dy, ddy] = getPoint(trajectory, t)
    
    x = trajectory.fx(t);
    dx = trajectory.dfx(t);
    ddx = trajectory.ddfx(t);
    y = trajectory.fy(t);
    dy = trajectory.dfy(t);
    ddy = trajectory.ddfy(t);
        
end

function [x, dx, ddx, y, dy, ddy] = getTrajectory(trajectory, tData)
    
    for i=1:length(tData)
        [x(i), dx(i), ddx(i), y(i), dy(i), ddy(i)] = trajectory.getPoint(trajectory, tData(i));
    end
end

function [ trajectory ] = setPol(trajectory, pol)
    trajectory.pol = pol;
    
    a = trajectory.pol(1,:);
    b = trajectory.pol(2,:);
    
    switch trajectory.degree
        case '1'
            trajectory.fx  = @(t) a(1)*t + a(2);
            trajectory.dfx = @(t) a(1);
            trajectory.ddfx = @(t) 0;
                
            trajectory.fy  = @(t) b(1)*t + b(2);
            trajectory.dfy = @(t) b(1);
            trajectory.ddfy = @(t) 0;
        case '2'
            trajectory.fx  = @(t) a(1)*t^2 + a(2)*t + a(3);
            trajectory.dfx = @(t) 2*a(1)*t + a(2);
            trajectory.ddfx = @(t) 2*a(1);
                
            trajectory.fy  = @(t) b(1)*t^2 + b(2)*t + b(3);
            trajectory.dfy = @(t) 2*b(1)*t + b(2);
            trajectory.ddfy = @(t) 2*b(1);
        case '3'
            trajectory.fx  = @(t) a(1)*t^3 + a(2)*t^2 + a(3)*t + a(4);
            trajectory.dfx = @(t) 3*a(1)*t^2 + 2*a(2)*t + a(3);
            trajectory.ddfx = @(t) 6*a(1)*t + 2*a(2);
                 
            trajectory.fy  = @(t) b(1)*t^3 + b(2)*t^2 + b(3)*t + b(4);
            trajectory.dfy = @(t) 3*b(1)*t^2 + 2*b(2)*t + b(3);
            trajectory.ddfy = @(t) 6*b(1)*t + 2*b(2);
         case '4'
            trajectory.fx  = @(t) a(1)*t^4 + a(2)*t^3 + a(3)*t^2 + a(4)*t + a(5);
            trajectory.dfx = @(t) 4*a(1)*t^3 + 3*a(2)*t^2 + 2*a(3)*t + a(4);
            trajectory.ddfx = @(t) 12*a(1)*t^2 + 6*a(2)*t + 2*a(3);
                
            trajectory.fy  = @(t) b(1)*t^4 + b(2)*t^3 + b(3)*t^2 + b(4)*t + b(5);
            trajectory.dfy = @(t) 4*b(1)*t^3 + 3*b(2)*t^2 + 2*b(3)*t + b(4);
            trajectory.ddfy = @(t) 12*b(1)*t^2 + 6*b(2)*t + 2*b(3);
        case '5'
            trajectory.fx  = @(t) a(1)*t^5 + a(2)*t^4 + a(3)*t^3 + a(4)*t^2 + a(5)*t + a(6);
            trajectory.dfx = @(t) 5*a(1)*t^4 + 4*a(2)*t^3 + 3*a(3)*t^2 + 2*a(4)*t + a(5);
            trajectory.ddfx = @(t) 20*a(1)*t^3 + 12*a(2)*t^2 + 6*a(3)*t + 2*a(4);
                
            trajectory.fy  = @(t) b(1)*t^5 + b(2)*t^4 + b(3)*t^3 + b(4)*t^2 + b(5)*t + b(6);
            trajectory.dfy = @(t) 5*b(1)*t^4 + 4*b(2)*t^3 + 3*b(3)*t^2 + 2*b(4)*t + b(5);
            trajectory.ddfy = @(t) 20*b(1)*t^3 + 12*b(2)*t^2 + 6*b(3)*t + 2*b(4);
        otherwise
            error('Wrong argument');
    end    
      
end

function [pol, err] = polyfit(trajectory, xData, yData, tData)

    % Objective function: Sum of squared errors
    f = @(theta, Y) sum((Y - polyval(theta,tData)).^2);

    % Options for minimizer
    opts = optimoptions('fminunc','Algorithm','quasi-newton');
%     opts = optimoptions('fminunc','Algorithm','trust-region');
    
    % Find best fiting parameters
    fx = @(theta)f(theta, xData); 
    fy = @(theta)f(theta, yData); 

    theta0 = rand(str2double(trajectory.degree)+1,1);
%     theta0 = -ones(str2double(trajectory.degree)+1,1);
    thetaX = fminunc(fx, theta0, opts);
    thetaY = fminunc(fy, theta0, opts);    
        
    pol = [thetaX'; thetaY'];
    
    err(1) = f(pol(1,:), xData);
    err(2) = f(pol(2,:), yData);
    
end


                       
