%Note: The x and y axis is interchange because the trajectory is in local
%car local coordinates. ISO 8855-2011
function plotTrajectory( carStates, polynomialTrajectoryStates )
    
    t = [carStates.t];

    figure('Name', 'Polynomial Trajectory', 'NumberTitle', 'off');
    fig = gcf;
    set(fig, 'Position',  [75, 75, 1500, 900]);

    subplot(2,2,1);
    ax1 = gca;
    title(ax1,'Vehicle Longitudinal Position');
    hold(ax1, 'on');
    grid(ax1, 'on');
    xlim(ax1, [0, 4]);
    ylim(ax1, [-1, 20]);
    xlabel(ax1, 'Time [s]');
    ylabel(ax1, 'x [m]');
    plot(ax1, t, [carStates.x], 'b.-');
    plot(ax1, t, polynomialTrajectoryStates.x, 'r.-');
    legend(ax1, 'ref', 'refPolyTraj');
    hold(ax1, 'off');
    
    subplot(2,2,2);
    ax2 = gca;
    title(ax2, 'Vehicle Lateral Position');
    hold(ax2, 'on');
    grid(ax2, 'on');
    xlim(ax2, [0, 4]);
    ylim(ax2, [-1, 10]);
    xlabel(ax2,'Time [s]');
    ylabel(ax2, 'y [m]');
    plot(ax2, t, [carStates.y], 'b.-');
    plot(ax2, t, polynomialTrajectoryStates.y, 'r.-');
    legend(ax2, 'ref', 'refPolyTraj');
    hold(ax2, 'off');

    subplot(2,2,3);
    ax3 = gca;
    title(ax3, 'Vehicle Longitudinal Velocity');
    hold(ax3, 'on');
    grid(ax3, 'on');
    xlim(ax3, [0, 4]);
%     ylim(ax3, [0, 20]);
    xlabel(ax3, 'Time [s]');
    ylabel(ax3, 'vx [m/s]');
    plot(ax3, t, [carStates.v_x], 'b.-');
    plot(ax3, t, polynomialTrajectoryStates.vx, 'r.-');
    legend(ax3, 'ref', 'refPolyTraj');
    hold('off');

    subplot(2,2,4);
    ax4 = gca;
    title(ax4, 'Vehicle Lateral Velocity');
    hold(ax4, 'on');
    grid(ax4, 'on');
    xlim(ax4, [0, 4]);
%     ylim(ax4, [0, 20]);
    xlabel(ax4, 'Time [s]');
    ylabel(ax4, 'vy [m/s]');
    plot(ax4, t, [carStates.v_y], 'b.-');
    plot(t, polynomialTrajectoryStates.vy, 'r.-');
    legend(ax4, 'ref', 'refPolyTraj');
    hold('off');

%     subplot(3,2,5);
%     ax5 = gca;
%     title(ax5, 'Vehicle Longitudinal Acceleration');
%     hold(ax5, 'on');
%     grid(ax5, 'on');
%     axis(ax5, 'equal');
%     xlabel(ax5, 'Time [s]');
%     ylabel(ax5, 'ax [m/s^2]');
%     plot(ax5, t, [gradient([carStates.v_y]) ./ gradient(t)], 'b.-');
%     plot(ax5, t, polynomialTrajectoryStates.ay, 'r.-');
%     legend(ax5, 'ref', 'refPolyTraj');
%     hold('off');
% 
%     subplot(3,2,6);
%     ax6 = gca;
%     title(ax6, 'Vehicle Lateral Acceleration');
%     hold(ax6, 'on');
%     grid(ax6, 'on');
%     axis(ax6, 'equal');
%     xlabel(ax6, 'Time [s]');
%     ylabel(ax6, 'ay [m/s^2]');
%     plot(ax6, t, [gradient([carStates.v_x]) ./ gradient(t)], 'b.-');
%     plot(ax6, t, polynomialTrajectoryStates.ax, 'r.-');
%     legend(ax6, 'ref', 'refPolyTraj');
%     hold('off');

end

