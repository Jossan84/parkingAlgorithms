function [polynomial] = Polynomial(varargin)
            
    for i = 1 :2: nargin
        switch varargin{i}
            case 'First order'
                degree = '1';
                a = [0 0];
                
                fx  = @(t) a(1)*t + a(2);
                dfx = @(t)a(1);
                ddfx = @(t)0;
                                
            case 'Second order'
                degree = '2';
                a = [0 0 0];
                
                fx  = @(t) a(1)*t^2 + a(2)*t + a(3);
                dfx = @(t) 2*a(1)*t + a(2);
                ddfx = @(t) 2*a(1);
                                             
            case 'Third order'
                degree = '3';
                a = [0 0 0 0];            

                fx  = @(t) a(1)*t^3 + a(2)*t^2 + a(3)*t + a(4);
                dfx = @(t) 3*a(1)*t^2 + 2*a(2)*t + a(3);
                ddfx = @(t) 6*a(1)*t + 2*a(2);
                                 
            case 'Fourth order'
                degree = '4';
                a = [0 0 0 0 0];
                
                fx  = @(t) a(1)*t^4 + a(2)*t^3 + a(3)*t^2 + a(4)*t + a(5);
                dfx = @(t) 4*a(1)*t^3 + 3*a(2)*t^2 + 2*a(3)*t + a(4);
                ddfx = @(t) 12*a(1)*t^2 + 6*a(2)*t + 2*a(3);
      
            case 'Fifth order'
                degree = '5';
                a = [0 0 0 0 0 0];
                
                fx  = @(t) a(1)*t^5 + a(2)*t^4 + a(3)*t^3 + a(4)*t^2 + a(5)*t + a(6);
                dfx = @(t) 5*a(1)*t^4 + 4*a(2)*t^3 + 3*a(3)*t^2 + 2*a(4)*t + a(5);
                ddfx = @(t) 20*a(1)*t^3 + 12*a(2)*t^2 + 6*a(3)*t + 2*a(4);
                
            otherwise
                error('Wrong argument');
        end
    end
    
    parameters = a;
    
    polynomial.parameters = parameters ;
    polynomial.degree = degree;
    polynomial.fx = fx;
    polynomial.dfx = dfx;
    polynomial.ddfx = ddfx;
    
    polynomial.getPoint = @getPoint;
    polynomial.getPolynomic = @getPolynomic;
    polynomial.setPol = @setPol;
    polynomial.polyfit = @polyfit;
    
end

function [x, dx, ddx] = getPoint(trajectory, t)
    
    x = trajectory.fx(t);
    dx = trajectory.dfx(t);
    ddx = trajectory.ddfx(t);
        
end

function [x, dx, ddx] = getPolynomic(polynomic, tData)
    
    for i=1:length(tData)
        [x(i), dx(i), ddx(i)] = polynomic.getPoint(polynomic, tData(i));
    end
end

function [ polynomic ] = setPol(polynomic, parameters)
    polynomic.parameters = parameters;
    
    a = polynomic.parameters;
    
    switch polynomic.degree
        case '1'
            polynomic.fx  = @(t) a(1)*t + a(2);
            polynomic.dfx = @(t) a(1);
            polynomic.ddfx = @(t) 0;
                
        case '2'
            polynomic.fx  = @(t) a(1)*t^2 + a(2)*t + a(3);
            polynomic.dfx = @(t) 2*a(1)*t + a(2);
            polynomic.ddfx = @(t) 2*a(1);
                
        case '3'
            polynomic.fx  = @(t) a(1)*t^3 + a(2)*t^2 + a(3)*t + a(4);
            polynomic.dfx = @(t) 3*a(1)*t^2 + 2*a(2)*t + a(3);
            polynomic.ddfx = @(t) 6*a(1)*t + 2*a(2);
                 
         case '4'
            polynomic.fx  = @(t) a(1)*t^4 + a(2)*t^3 + a(3)*t^2 + a(4)*t + a(5);
            polynomic.dfx = @(t) 4*a(1)*t^3 + 3*a(2)*t^2 + 2*a(3)*t + a(4);
            polynomic.ddfx = @(t) 12*a(1)*t^2 + 6*a(2)*t + 2*a(3);
                
        case '5'
            polynomic.fx  = @(t) a(1)*t^5 + a(2)*t^4 + a(3)*t^3 + a(4)*t^2 + a(5)*t + a(6);
            polynomic.dfx = @(t) 5*a(1)*t^4 + 4*a(2)*t^3 + 3*a(3)*t^2 + 2*a(4)*t + a(5);
            polynomic.ddfx = @(t) 20*a(1)*t^3 + 12*a(2)*t^2 + 6*a(3)*t + 2*a(4);
                
        otherwise
            error('Wrong argument');
    end  
    
end

function [parameters, err] = polyfit(polynomic, xData, tData)

    % Objective function: Sum of squared errors
    f = @(theta, Y) sum((Y - polyval(theta,tData)).^2);

    % Options for minimizer
    opts = optimoptions('fminunc','Algorithm','quasi-newton');
%     opts = optimoptions('fminunc','Algorithm','trust-region');
    
    % Find best fiting parameters
    fx = @(theta)f(theta, xData); 
    
    theta0 = rand(str2double(polynomic.degree)+1,1);
%     theta0 = zeros(str2double(polynomic.degree)+1,1)';
    theta = fminunc(fx, theta0, opts);
       
    parameters = theta';
    
    err = f(parameters, xData);
    
end


                       
