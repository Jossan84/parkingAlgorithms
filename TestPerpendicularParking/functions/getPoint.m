function [ goalPoint ] = getPoint( Path_L, pointDistance )
    
    Q1 = Path_L(:,2);
    Q2 = Path_L(:,1);

    Lp = norm(Q1'-Q2'); % Distance between points
    cosGamma = ((norm(Q2)^2 - norm(Q1)^2 - Lp^2)/(-2*Lp*norm(Q1))); 
    P = norm(Q1) * cosGamma + sqrt( norm(Q1)^2 * ((cosGamma^2)-1) + pointDistance^2); 
   
    goalPoint = (P * [(Q2(1)-Q1(1))/Lp ;(Q2(2)-Q1(2))/Lp]) + Q1;

end

