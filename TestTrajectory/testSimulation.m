%testSimulation
%09/07/2020

close all;
clear;
more off;
clc;

if exist('OCTAVE_VERSION', 'builtin') ~= 0% OCTAVE
    IDE = 'OCTAVE';
    markerSize = 12;
else% MATLAB
    IDE = 'MATLAB';
    markerSize = 6;
end

addpath('classes');
addpath('functions');

T = 40e-3;
N = 100;

% Desired speed
u = 20/3.6 * ones(N,1); % [m/s] 

% Desired path
R = 30; 
[path_G] = generateArcPath(R);

% Get data
[virtualCarStates, delta] = getVirtualCarSimulation('T', T,...
                                                 'N', N,...
                                                 'vx', u,...
                                                 'path', path_G,...
                                                 'trajectory','circular'); 

% Plot simulation
figure;
ax = gca;   
title(ax,'Vehicle Trajectory');
hold(ax,'on');
grid(ax,'on');
axis(ax,'equal');
plot(ax, [virtualCarStates(:).x], [virtualCarStates(:).y],'b.-', 'MarkerSize',12);
plot(ax, path_G(:,1), path_G(:,2),'k.-');
hold(ax,'off');

figure;
ax = subplot(5,1,1);
plot(ax, [virtualCarStates(:).t], [virtualCarStates(:).v_x], 'b.-');
hold(ax, 'on');
plot(ax, [virtualCarStates(:).t], mean([virtualCarStates(:).v_x])*ones(N+1,1), 'r.-');
grid(ax,'on');
xlabel(ax, 't [m/s]');
ylabel(ax, 'a [m/s]');
ax1 = subplot(5,1,2);
plot(ax1, [virtualCarStates(:).t], [0 diff([virtualCarStates(:).v_x])], 'b.-');
hold(ax1, 'on');
grid(ax1,'on');
plot(ax1, [virtualCarStates(:).t], mean([0 diff([virtualCarStates(:).v_x])])*ones(N+1,1), 'r.-');
xlabel(ax1, 't [m/s]');
ylabel(ax1, 'a [m/s]');
ax2 = subplot(5,1,3);
plot(ax2, [virtualCarStates(:).t], [virtualCarStates(:).x], 'b.-');
grid(ax2,'on');
xlabel(ax2, 't [m/s]');
ylabel(ax2, 'x [m/s]');
ax3 = subplot(5,1,4);
plot(ax3, [virtualCarStates(:).t], [virtualCarStates(:).y], 'b.-');
grid(ax3,'on');
xlabel(ax3, 't [m/s]');
ylabel(ax3, 'y [m/s]');
ax4 = subplot(5,1,5);
plot(ax4, [virtualCarStates(:).t], [virtualCarStates(:).yawRate]./[virtualCarStates(:).v_x], 'b.-');
grid(ax4,'on');
xlabel(ax4, 't [m/s]');
ylabel(ax4, 'c [m^-1]');


rmpath('classes');
rmpath('functions');
