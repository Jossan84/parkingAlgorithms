function plotCurvature(t, kRef, kPolyTraj, kPolyCurv, rRef, rPolyTraj, rPolyCurv, deltaSRef, deltaSPolyTraj, deltaSPolyCurv)
        
    figure('Name', 'Polynomial Curvature', 'NumberTitle', 'off');
    fig = gcf;
    set(fig, 'Position',  [75, 75, 1100, 900]);

    subplot(4,1,1);
    ax1 = gca;
    title(ax1,'Curvature');
    hold(ax1, 'on');
    grid(ax1, 'on');
    xlabel(ax1, 'Time [s]');
    ylabel(ax1, 'k [rad/m]');
    plot(ax1, t, kRef, 'b.-');
    plot(ax1, t, kPolyTraj, 'r.-');
    plot(ax1, t, kPolyCurv, 'k.-');
    %ylim(ax1, [min(kPolyTraj) max(kPolyTraj)]);
    legend(ax1, 'ref', 'RefPolyTraj', 'RefPolyCurv');
    hold(ax1, 'off');
    
    subplot(4,1,2);
    ax1 = gca;
    title(ax1,'Curvature Error');
    hold(ax1, 'on');
    grid(ax1, 'on');
    xlabel(ax1, 'Time [s]');
    ylabel(ax1, 'k error');
    plot(ax1, t, (abs(kRef) - abs(kPolyTraj)).^2, 'r.-');
    plot(ax1, t, (abs(kRef) - abs(kPolyCurv)).^2, 'k.-');
%     ylim(ax1, [min(kPolyTraj) max(kPolyTraj)]);
    legend(ax1, 'RefPolyTraj', 'RefPolyCurv');
    hold(ax1, 'off');
    
    subplot(4,1,3);
    ax2 = gca;
    title(ax2,'Turning Radius');
    hold(ax2, 'on');
    grid(ax2, 'on');
    xlabel(ax2, 'Time [s]');
    ylabel(ax2, 'r [m]');
    plot(ax2, t, rRef, 'b.-');
    plot(ax2, t, rPolyTraj, 'r.-');
    plot(ax2, t, rPolyCurv, 'k.-');
    legend(ax2, 'ref', 'RefPolyTraj', 'RefPolyCurv');
    hold(ax2, 'off');

    subplot(4,1,4);
    ax3 = gca;
    title(ax3,'Steering Wheel Angle');
    hold(ax3, 'on');
    grid(ax3, 'on');
    xlabel(ax3, 'Time [s]');
    ylabel(ax3, '[deg]');
    plot(ax3, t, deltaSRef, 'b.-');
    plot(ax3, t, deltaSPolyTraj, 'r.-');
    plot(ax3, t, deltaSPolyCurv, 'k.-');
    legend(ax3, 'ref', 'RefPolyTraj', 'RefPolyCurv');
    hold(ax3, 'off');


end

