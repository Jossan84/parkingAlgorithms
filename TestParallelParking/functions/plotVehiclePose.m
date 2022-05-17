% plotVehiclePose.m
% 11/02/2020

function plotVehiclePose(ax, x, y, yaw)

    scaleFactor = 1;
    
    quiver(ax, x, y, scaleFactor * cos(yaw), scaleFactor * sin(yaw), 'b', 'LineWidth', 2, 'AutoScale', 'off');
    hold on
    plot(ax,x, y,'bo','MarkerSize',5,'MarkerFaceColor',[0.25,0.25,0.25]);
    
end