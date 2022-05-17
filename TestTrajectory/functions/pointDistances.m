function [ y ] = pointDistances( x , pointDistance )

[n, m] = size(x);

for i=1:m
   
y(i) = norm([x(1,i),x(2,i)],2) - pointDistance;

end

end

