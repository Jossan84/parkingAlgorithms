function [goalPoint, index] = getLookAheadPoint(varargin)

  for i = 1 :2: nargin
        switch varargin{i}
            case 'IDE'
                IDE = varargin{i + 1};
            case 'Path_L'
                Path_L = varargin{i + 1};
            case 'pointDistance'
                pointDistance = varargin{i + 1};    
            otherwise
                error('Wrong argument');
        end
   end
    
   if IDE == 'OCTAVE'
      [~, minDistIndex] = min((norm(Path_L, 2, 'columns') - pointDistance).^2);% OCTAVE 
   else     
      [~, minDistIndex] = min(pointDistances(Path_L,pointDistance).^2);        % MATLAB 2016a
   end
   
%% Search Goal Point     
%    %Searched points
   p0 = Path_L(:, minDistIndex-1);
   p1 = Path_L(:, minDistIndex);
   p2 = Path_L(:, minDistIndex+1);
       
   d0 = norm(p0);
   d1 = norm(p1);
   d2 = norm(p2);
   
   points.x = [p0(1), p1(1), p2(1)];
   points.y = [p0(2), p1(2), p2(2)];
   err = [pointDistance - d0, pointDistance - d1, pointDistance - d2];
   [~, maxIndex] = max(err);
   [~, minIndex] = min(err);
   Q2 = [points.x(minIndex); points.y(minIndex)];
   Q1 = [points.x(maxIndex); points.y(maxIndex)];
   index = minDistIndex;
      
%    if d0 <= pointDistance && d1 > pointDistance && d2 >= d1
%        Q1 = p0;
%        Q2 = p1;
%        index = minDistIndex - 1;
%    elseif d1 <= pointDistance && d2 > pointDistance
%        Q1 = p1;
%        Q2 = p2;
%        index = minDistIndex;
%    end
   
   Lp = norm(Q1'-Q2'); % Distance between points
   cosGamma = ((norm(Q2)^2 - norm(Q1)^2 - Lp^2)/(-2*Lp*norm(Q1))); 
   P = norm(Q1) * cosGamma + sqrt( norm(Q1)^2 * ((cosGamma^2)-1) + pointDistance^2); 
   
   goalPoint = (P * [(Q2(1)-Q1(1))/Lp ;(Q2(2)-Q1(2))/Lp]) + Q1;
   
  
%   goalPoint = Path_L(:, minDistIndex);
%   index = minDistIndex;

end

