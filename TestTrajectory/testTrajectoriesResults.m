%testTajectoriesResults
%9/07/2020

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
addpath('results');

% Test parameters:
N = 100;
rMax = 1000; % [m]
u = 20/3.6 * ones(N,1); % [m/s] 
T = 40e-3;
R = [30: 10 : rMax]';

varNames = {'radius',...
            'fxError1', 'fyError1',...
            'fxError2', 'fyError2',...
            'fxError3', 'fyError3',...
            'fxError4', 'fyError4',...
            'fxError5', 'fyError5'};
results = table(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 'VariableNames', varNames); 

for i = 1 : length( R )
    [fxError, fyError, trajectory] = getTestTrajectoriesResults('T', T, 'N', N, 'R', R(i), 'vx', u);
    results.radius(i) = R(i);
    results.fxError1(i) = fxError{1}(:);
    results.fxError2(i) = fxError{2}(:);
    results.fxError3(i) = fxError{3}(:);
    results.fxError4(i) = fxError{4}(:);
    results.fxError5(i) = fxError{5}(:);
    results.fyError1(i) = fyError{1}(:);
    results.fyError2(i) = fyError{2}(:);
    results.fyError3(i) = fyError{3}(:);
    results.fyError4(i) = fyError{4}(:);
    results.fyError5(i) = fyError{5}(:);
end

filesNum = numel(dir(['results', '/*.xls'])) + 1;

fileName = ['results\',['results',num2str(20),'Kmh',num2str(filesNum)],'.xls'];                   
writetable(results, fileName, 'WriteRowNames', true);   

rmpath('classes');
rmpath('functions');
rmpath('results');
