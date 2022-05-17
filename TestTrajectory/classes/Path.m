function [path] = Path(varargin)
            
    for i = 1 :2: nargin
        switch varargin{i}
            case 'Bezier'
                degree = '1';
                n = 1;
                p0 = [ -3.5 -3.5   100; 0   100 100];   % [x;y] [m];
                p1 = [ -3.5 100 100; 100 100 200];   %  " "   "
%                 p0 = [ 0 ; 0  ];   % [x;y] [m];
%                 p1 = [ 0 ; 100];   %  " "   "
                p2 = [];
                p3 = [];
                p4 = [];
                p5 = [];
            case 'Bezier2'
                degree = '2';
                n = 2;
                p0 = [ -3.5 -3.5   100; 0   100 100];   % [x;y] [m];
                p1 = [ -3.5 50  100; 50  100 150];   %  " "   " 
                p2 = [ -3.5 100 100; 100 100 200];   %  " "   "
%                 p0 = [ 0 ; 0  ];   % [x;y] [m];
%                 p1 = [ 0 ; 50 ];   %  " "   " 
%                 p2 = [ 0 ; 100];   %  " "   "
                p3 = [];
                p4 = [];
                p5 = [];
            case 'Bezier3'
                degree = '3';
                n = 3;
%                 p0 = [ 0 0   100; 0   100 100];   % [x;y] [m];
%                 p1 = [ 0 25  100; 25  100 125];   %  " "   " 
%                 p2 = [ 0 75  100; 75  100 175];   %  " "   " 
%                 p3 = [ 0 100 100; 100 100 200];   %  " "   "
                p0 = [ -3.5 -3.5   100; 0   100 100];   % [x;y] [m];
                p1 = [ -3.5 25  100; 25  100 125];   %  " "   " 
                p2 = [ -3.5 75  100; 75  100 175];   %  " "   " 
                p3 = [ -3.5 100 100; 100 100 200];   %  " "   "
                p4 = [];
                p5 = [];
            case 'Bezier5'
                degree = '5';
                n = 5;
                p0 = [ -3.5 -3.5   100; 0   100 100];   % [x;y] [m];
                p1 = [ -3.5 20  100; 20  100 120];   %  " "   " 
                p2 = [ -3.5 40  100; 40  100 140];   %  " "   " 
                p3 = [ -3.5 60  100; 60  100 160];   %  " "   " 
                p4 = [ -3.5 80  100; 80  100 180];   %  " "   " 
                p5 = [ -3.5 100 100; 100 100 200];   %  " "   "
%                 p0 = [ 0 ; 0  ];   % [x;y] [m];
%                 p1 = [ 0 ; 20 ];   %  " "   " 
%                 p2 = [ 0 ; 40 ];   %  " "   " 
%                 p3 = [ 0 ; 60 ];   %  " "   " 
%                 p4 = [ 0 ; 80 ];   %  " "   " 
%                 p5 = [ 0 ; 100];   %  " "   "
            otherwise
                error('Wrong argument');
        end
    end
    
    points = [p0 p1 p2 p3 p4 p5];
    
    path.points                 = points;
    path.degree                 = degree;
    path.n                      = n;
    
    path.getPath                = @getPath;
    path.getCrossTrackError     = @getCrossTrackError;
    path.getLookAheadPoint      = @getLookAheadPoint;
    path.plotPath               = @plotPath;
    path.getBezierPoints        = @getBezierPoints;
    path.setPathPoints          = @setPathPoints;
    path.getPathPoints          = @getPathPoints;
    
end

function [x, y] = getPath(path,t,segment)

        switch path.degree
            case '1'
                P0 = path.points(:,segment);
                P1 = path.points(:,segment+path.n);
                x  = @(t) (1-t)*P0(1) + t*P1(1);
                y  = @(t) (1-t)*P0(2) + t*P1(2);
                x  = x(t);
                y  = y(t);                
                
            case '2'
                P0 = path.points(:,segment);
                P1 = path.points(:,segment+path.n);
                P2 = path.points(:,segment+path.n*2);
                x = @(t) ((1-t).^2)*P0(1) + 2*(1-t).*t*P1(1) + (t.^2)*P2(1);
                y = @(t) ((1-t).^2)*P0(2) + 2*(1-t).*t*P1(2) + (t.^2)*P2(2);
                x = x(t);
                y = y(t);
                
            case '3'
                P0 = path.points(:,segment);
                P1 = path.points(:,segment+path.n);
                P2 = path.points(:,segment+path.n*2);
                P3 = path.points(:,segment+path.n*3);
                x = @(t) ((1-t).^3)*P0(1) + 3*((1-t).^2).*t*P1(1) + 3*(1-t).*...
                         (t.^2)*P2(1) + (t.^3)*P3(1);
                y = @(t) ((1-t).^3)*P0(2) + 3*((1-t).^2).*t*P1(2) + 3*(1-t).*...
                         (t.^2)*P2(2) + (t.^3)*P3(2);
                x = x(t);
                y = y(t);
                
            case '5'
                P0 = path.points(:,segment);
                P1 = path.points(:,segment+path.n);
                P2 = path.points(:,segment+path.n*2);
                P3 = path.points(:,segment+path.n*3);
                P4 = path.points(:,segment+path.n*4);
                P5 = path.points(:,segment+path.n*5);
                x = @(t) ((1-t).^5)*P0(1) + 5*((1-t).^4).*t*P1(1) + 10*((1-t).^3).*...
                         (t.^2)*P2(1) + 10*((1-t).^2).*(t.^3)*P3(1) + 5*(1-t).*...
                         (t.^4)*P4(1) + (t.^5)*P5(1);
                y = @(t) ((1-t).^5)*P0(2) + 5*((1-t).^4).*t*P1(2) + 10*((1-t).^3).*...
                         (t.^2)*P2(2) + 10*((1-t).^2).*(t.^3)*P3(2) + 5*(1-t).*...
                         (t.^4)*P4(2) + (t.^5)*P5(2);
                x = x(t);
                y = y(t);
            otherwise
                error('Wrong argument');
        end
end

function [] = plotPath(path,nSteps)

    t      = linspace(0,1,nSteps);
    
    x = [];
    y = [];
    
    for i=1:path.n
        [x_i, y_i] = path.getPath(path,t,i);       
        x = [x x_i];
        y = [y y_i];
    end    
    
    %figure(1);
    plot(x,y,'k.-');
    title('Path');
    ylabel('y [m]');
    xlabel('x [m]');
    
end
function [cte,i,t,nIter] = getCrossTrackError(path,x0,y0,R,iBezier,tBezier,maxDistance,maxIter,method)

    i = iBezier;
    t = tBezier;
    cte = 0;
                    
    [x,dx,ddx,y,dy,ddy]= path.getBezierPoints(path,iBezier,tBezier);
                    
    % Objective function and derivatives to minimization
    f1   = ((x-x0)^2) + ((y-y0)^2) - R^2;
    df1  = 2*(x-x0) * dx  + 2*(y-y0) * dy;
    ddf1  = 2*(dx*dx + (x-x0)*ddx) + 2*(dy*dy + (y-y0)*ddy);
    
    f2   = f1 * f1;
    df2  = 2 * f1 * df1;
    ddf2 = 2 * ((df1*df1) + (f1 * ddf1));
              
    nIter = 0;
        switch method
            case 'Newton'                
                while((abs(df2) > maxDistance^4) && (nIter < maxIter))
                                
                    t = t - (df2/ddf2); % Update Newton
                                
                    if(t<0 && i>0)
                        i = i - 1;
                        t = t - floor(t);   
                    elseif(t > 1 && i < (path.n - 1))
                        i = i + 1;
                        t = t - floor(t);
                    end
                                
                    [x,dx,ddx,y,dy,ddy]= path.getBezierPoints(path,i,t);
                    
                    % Objective function and derivatives to minimization
                    f1   = ((x-x0)^2) + ((y-y0)^2) - R^2;
                    df1  = 2*(x-x0) * dx  + 2*(y-y0) * dy;
                    ddf1  = 2*(dx*dx + (x-x0)*ddx) + 2*(dy*dy + (y-y0)*ddy);
                    
                    f2   = f1 * f1;
                    df2  = 2 * f1 * df1;
                    ddf2 = 2 * ((df1*df1) + (f1 * ddf1));
                    
                    nIter = nIter + 1;     
                end
                            
                if(nIter < maxIter)
                    cte = sqrt(sqrt(f2));
                else    
                    cte = NaN;
                end
                                         
        case 'Bisection'
            disp('Bisection is not implemented');
        case 'RegulaFalsi'
            disp('RegulaFalsi is not implemented');
        otherwise
            error('Method not valid');   
        end
                       
end

function [goalPoint, isFound, nIter] = getLookAheadPoint(path,x0,y0,R,iBezier,tBezier,maxDistance,maxIter,method)
            
    i = iBezier;
    t = tBezier;
    isFound = 0;
                    
    [x,dx,ddx,y,dy,ddy]= path.getBezierPoints(path,iBezier,tBezier);
                    
    % Objective function and derivatives to minimization
    f1   = ((x-x0)^2) + ((y-y0)^2) - R^2;
    df1  = 2*(x-x0) * dx  + 2*(y-y0) * dy;
    ddf1  = 2*(dx*dx + (x-x0)*ddx) + 2*(dy*dy + (y-y0)*ddy);
              
    nIter = 0;
        switch method
            case 'Newton'                
                while((abs(f1) > maxDistance^2) && (nIter < maxIter))
                                
                    t = t - (f1/df1); % Update Newton
%                    [i, t] = tToIntegerAndDecimal(t + iBezier, path.n);
                    
                    if(t<0 && i>0)
                        i = i - 1;
                        t = t - floor(t);   
                    elseif(t > 1 && i < (path.n - 1))
                        i = i + 1;
                        t = t - floor(t);
                    end
                                
                    [x,dx,ddx,y,dy,ddy]= path.getBezierPoints(path,i,t);
                    
                    % Objective function and derivatives to minimization
                    f1   = ((x-x0)^2) + ((y-y0)^2) - R^2;
                    df1  = 2*(x-x0) * dx  + 2*(y-y0) * dy;
                    ddf1  = 2*(dx*dx + (x-x0)*ddx) + 2*(dy*dy + (y-y0)*ddy);
                                
                    nIter = nIter + 1;     
                end
                            
                if(nIter < maxIter)
                    isFound = 1;
                    goalPoint(1) = x;
                    goalPoint(2) = y;                          
                end
                                  
        case 'Bisection'
            disp('Bisection is not implemented');
        case 'RegulaFalsi'
            disp('RegulaFalsi is not implemented');
        otherwise
            error('Method not valid');   
        end
                       
end

function [x,dx,ddx,y,dy,ddy] = getBezierPoints(path,i,t)

    switch path.degree
            case '1'

                    P0 = path.points(:,i);
                    P1 = path.points(:,i+path.n);
                
                    x   = ((1-t)*P0(1) + t*P1(1));
                    dx  = (P1(1) - P0(1));
                    ddx = (0);
                
                    y   = ((1-t)*P0(2) + t*P1(2));
                    dy  = (P1(2) - P0(2));
                    ddy = (0);
                    
           case '2'

                    P0  = path.points(:,i);
                    P1  = path.points(:,i+path.n);
                    P2  = path.points(:,i+path.n*2);

                    x   = ((1-t).^2)*P0(1) + 2*(1-t).*t*P1(1) + (t.^2)*P2(1);
                    dx  = (2*P2(1).*t - 2*P1(1).*t + P0(1)*(2.*t - 2) - P1(1)*(2.*t - 2));
                    ddx = (2*P0(1) - 4*P1(1) + 2*P2(1));

                    y   = ((1-t).^2)*P0(2) + 2*(1-t).*t*P1(2) + (t.^2)*P2(2);
                    dy  = (2*P2(2).*t - 2*P1(2).*t + P0(2)*(2.*t - 2) - P1(2)*(2.*t - 2));
                    ddy = (2*P0(2) - 4*P1(2) + 2*P2(2));
                    
            case '3'
                    P0  = path.points(:,i);
                    P1  = path.points(:,i+path.n);
                    P2  = path.points(:,i+path.n*2);
                    P3  = path.points(:,i+path.n*3);
                    
                    x   = ((1-t).^3)*P0(1) + 3*((1-t).^2).*t*P1(1) + 3*(1-t).*(t.^2)*P2(1)...
                                + (t.^3)*P3(1);
                    dx  = (3*P3(1).*t.^2 - 3*P2(1).*t.^2 - 3*P0(1).*(t - 1).^2 + 3*P1(1).*...
                          (t - 1).^2 + 3*P1(1).*t*(2*t - 2) - 2*P2(1)*t*(3*t - 3));
                    ddx = (6*P1(1).*t - 12*P2(1).*t + 6*P3(1).*t - 3*P0(1)*(2.*t - 2) + ...
                          6*P1(1)*(2.*t - 2) - 2*P2(1)*(3*t - 3));

                    y   = ((1-t).^3)*P0(2) + 3*((1-t).^2).*t*P1(2) + 3*(1-t).*(t.^2)*P2(2) +...
                          (t.^3)*P3(2);
                    dy  = (3*P3(2).*t.^2 - 3*P2(2).*t.^2 - 3*P0(2).*(t - 1).^2 + 3*P1(2).*...
                          (t - 1).^2 + 3*P1(2).*t*(2*t - 2) - 2*P2(2)*t*(3*t - 3));
                    ddy = (6*P1(2).*t - 12*P2(2).*t + 6*P3(2).*t - 3*P0(2)*(2.*t - 2) +...
                          6*P1(2)*(2.*t - 2) - 2*P2(2)*(3*t - 3));                                         
            case '5'                
                    P0  = path.points(:,i);
                    P1  = path.points(:,i+path.n);
                    P2  = path.points(:,i+path.n*2);
                    P3  = path.points(:,i+path.n*3);
                    P4  = path.points(:,i+path.n*4);
                    P5  = path.points(:,i+path.n*5);
                    x   = ((1-t).^5)*P0(1) + 5*((1-t).^4).*t*P1(1) + 10*((1-t).^3).*(t.^2)...
                          *P2(1) + 10*((1-t).^2).*(t.^3)*P3(1) + 5*(1-t).*(t.^4)*P4(1)+...
                          (t.^5)*P5(1);
                    dx  = (5*P5(1)*t^4 - 5*P4(1)*t^4 - 5*P0(1)*(t - 1)^4 + 5*P1(1)*(t - 1)^4 ...
                          + 20*P1(1)*t*(t - 1)^3 - 20*P2(1)*t*(t - 1)^3 + 10*P3(1)*t^3* ...
                          (2*t - 2) - 4*P4(1)*t^3*(5*t - 5)- 30*P2(1)*t^2*(t - 1)^2 + ...
                          30*P3(1)*t^2*(t - 1)^2);
                    ddx = (5*P5(1)*t^4 - 5*P4(1)*t^4 - 5*P0(1)*(t - 1)^4 + 5*P1(1)*(t - 1)^4 ...
                          + 20*P1(1)*t*(t - 1)^3 - 20*P2(1)*t*(t - 1)^3 + 10*P3(1)*t^3* ...
                          (2*t - 2) - 4*P4(1)*t^3*(5*t - 5) - 30*P2(1)*t^2*(t - 1)^2 + ...
                          30*P3(1)*t^2*(t - 1)^2);       


                    y   = ((1-t).^5)*P0(2) + 5*((1-t).^4).*t*P1(2) + 10*((1-t).^3).*(t.^2)*P2(2)...
                          + 10*((1-t).^2).*(t.^3)*P3(2) + 5*(1-t).*(t.^4)*P4(2) + (t.^5)*P5(2);
                    dy  = (5*P5(2)*t^4 - 5*P4(2)*t^4 - 5*P0(2)*(t - 1)^4 + 5*P1(2)*(t - 1)^4 ...
                          + 20*P1(2)*t*(t - 1)^3 - 20*P2(2)*t*(t - 1)^3 + 10*P3(2)*t^3* ...
                          (2*t - 2) - 4*P4(2)*t^3*(5*t - 5)- 30*P2(2)*t^2*(t - 1)^2 + ...
                          30*P3(2)*t^2*(t - 1)^2);
                    ddy = (5*P5(2)*t^4 - 5*P4(2)*t^4 - 5*P0(2)*(t - 1)^4 + 5*P1(2)*(t - 1)^4 ...
                          + 20*P1(2)*t*(t - 1)^3 - 20*P2(2)*t*(t - 1)^3 + 10*P3(2)*t^3* ...
                          (2*t - 2) - 4*P4(2)*t^3*(5*t - 5) - 30*P2(2)*t^2*(t - 1)^2 ...
                          + 30*P3(2)*t^2*(t - 1)^2);
    end           
end

function [path] = setPathPoints(path,points)

    path.points = points;

end

function [points] = getPathPoints(path)

    points = path.points;

end
