%plotSimulation
%18/02/2020


close all;
clc;

figure;
ax = gca;   
hold(ax,'on');
    
for i=1:n-1
    
    cla(ax);
    show(parkingMap.map);
    title(ax,'Perpendicular Parking Setting');
    plot(ax,parkingSpotLocation.x ,parkingSpotLocation.y ,'b*');
    plotVehiclePose(ax, [carStates(i).x], [carStates(i).y], [carStates(i).yaw]);
    plot(ax,carContour(i).Points(1,:),carContour(i).Points(2,:),'b.-');
    
    pause(T);
    
end

    plot(ax, [carStates(2:end-1).x], [carStates(2:end-1).y],'r.-');
    plot(ax, [PathForward.x], [PathForward.y],'--k');
    plot(ax, [PathBackward.x], [PathBackward.y],'--k');

