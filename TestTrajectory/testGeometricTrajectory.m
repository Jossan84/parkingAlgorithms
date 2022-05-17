%testGeometricTrajectory
%26/11/2020

close all;
clear;
more off;
clc;

addpath('classes');
addpath('functions');

% Desired path
xc = 0; % [m]
yc = 30; % [m]
r = 30; % [m]
theta0 = 0; % [rad]

% Reference circular path
th = linspace(0, pi/2, 100);
xC =  flip(0 + r * cos(th));
yC =  flip(r -  r* sin(th));

%% Parametrization with the distance along the path.
L = 15; % [m]
s = linspace(0, L, 100); % distance vector [m]
dt = 1; % [s]
dx = s./dt;
omega = dx./r;
x0 = 0;
y0 = 0;
for i = 1 : length(s)
    x(i) = xc + r * cos(omega(i) + theta0 - pi/2);
    y(i) = yc + r * sin(omega(i) + theta0 - pi/2);    
end

% Aproximate geometric path to a polynomial function
pathG.x = x;
pathG.y = y;

% Get geometric polynomic path
degree = 5;
px = polyfit(dx, pathG.x, degree);
py = polyfit(dx, pathG.y, degree);

% Evaluation of the geometric polynomic path
T = 0.04;
t = (0 : T : 4)';
vx(1) = 0/3.6;
s1(1) = 0; 
numDelaySteps = 3;
for i = 1 : length(t)-1
    if i <= numDelaySteps 
        vx(i+1) = vx(i);
        s1(i+1) = s(i);
    else    
        vx(i+1) = transferFunction(20/3.6, vx(i), 0.90, 0.10);
        s1(i+1) = s1(i) + vx(i+1) * T;
    end
end

x1 = polyval(px, s1);
y1 = polyval(py, s1);

%% Parametrization with the time
% Get polynomic path dependent of the time
% y = theta0 + theta1*t^1 + theta2*t^2 + theta2*t^3 + theta2*t^4 + theta2*t^5
f = @(theta,t) theta(1)*t.^0 + theta(2)*t.^1 + theta(3)*t.^2 + theta(4)*t.^3 + theta(5)*t.^4 + theta(6)*t.^5;
X = [zeros(length(t),1), t.^1, t.^2, t.^3, t.^4, t.^5];
px2 = flip(normalEqn(X, x1'));
py2 = flip(normalEqn(X, y1'));

% degree = 5;
% px2 = polyfit(t, x1, degree);
% py2 = polyfit(t, y1, degree);

% Evaluation of the polynomic path with the time.
x2 = polyval(px2, t);
y2 = polyval(py2, t);


figure(1);
ax = gca;   
title(ax,'Vehicle Trajectory');
hold(ax,'on');
grid(ax,'on');
axis(ax,'equal');
xlabel(ax, 'x[m]');
ylabel(ax, 'y[m]');
plot(ax, xC, yC,'k');
% plot(ax, x, y,'k.-');
plot(ax, x1, y1,'b.-');
plot(ax, x2, y2,'r.-');
hold(ax,'off');

figure('Name', 'Polynomial Trajectory', 'NumberTitle', 'off');
fig = gcf;
set(fig, 'Position',  [75, 75, 1500, 900]);

subplot(6,1,1);
ax1 = gca;   
title(ax1,'vx');
hold(ax1,'on');
grid(ax1,'on');
% ylim(ax1,[0 6]);
% xlim(ax1,[0 4]);
stairs(ax1, t, vx,'b.-');
xlabel(ax1, 'Time [s]');
ylabel(ax1, 'vx [m/s]');
% hold(ax1,'off');

subplot(6,1,2);
ax2 = gca;   
title(ax2,'ax');
hold(ax2,'on');
grid(ax2,'on');
% stairs(ax2, t, [0 diff(vx)]./T,'b.-');
stairs(ax2, t, gradient(vx, T), 'b.-');
xlabel(ax2, 'Time [s]');
ylabel(ax2, 'ax [m/s^2]');
hold(ax2,'off');

subplot(6,1,3);
ax3 = gca;   
title(ax3,'s');
hold(ax3,'on');
grid(ax3,'on');
stairs(ax3, t, s1,'b.-');
xlabel(ax3, 'Time [s]');
ylabel(ax3, 's [m]');
hold(ax3,'off');

subplot(6,1,4);
ax4 = gca;   
title(ax4,'c');
hold(ax4,'on');
grid(ax4,'on');
ylim(ax4,[0.032 0.034]);
% xlim(ax1,[0 4]);
stairs(ax4, t, 1./r*ones(length(t),1),'b.-');
xlabel(ax4, 'Time [s]');
ylabel(ax4, 'c [m^-1]');
hold(ax4,'off');

subplot(6,1,5);
ax5 = gca;   
title(ax5,'x');
hold(ax5,'on');
grid(ax5,'on');
stairs(ax5, t, x1,'b.-');
stairs(ax5, t, x2,'r.-');
xlabel(ax5, 'Time [s]');
ylabel(ax5, 'x [m]');
hold(ax5,'off');

subplot(6,1,6);
ax6 = gca;   
title(ax6,'y');
hold(ax6,'on');
grid(ax6,'on');
stairs(ax6, t, y1,'b.-');
stairs(ax6, t, y2,'r.-');
xlabel(ax6, 'Time [s]');
ylabel(ax6, 'y [m]');
hold(ax6, 'off');


rmpath('classes');
rmpath('functions');
