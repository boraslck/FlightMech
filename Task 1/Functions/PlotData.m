%% PlotData.m
% Convert current states and controls into presentable units (e.g. radians to degrees) and generate plots of all variables relevant to the manoeuvre.

function PlotData(Eulers,Control,Time,X)
    % Convert back from radians to degrees where necessary
    Eulers = rad2deg(Eulers);
    Control = rad2deg(Control);

    % Plot horizontal v vertical position
    figure (1)
    hold on
    box on
    grid on
    set(gca,'FontSize',18,'FontName','Times New Roman');
    plot(X(11,:),X(12,:)) % Plot horizontal vs vertical position
    xlabel('Horizontal Position (m)');
    ylabel('Vertical Position (m)');
    hold off
    
    % Plotting euler angles
    figure (2)
    hold on
    box on
    grid on
    set(gca,'FontSize',18,'FontName','Times New Roman');
    plot(Time,Eulers(1,:), 'linewidth', 2)
    plot(Time,Eulers(2,:), 'linewidth', 2)
    plot(Time,Eulers(3,:), 'linewidth', 2)
    xlabel('Time (seconds)');
    ylabel('Euler Angles ({\circ})');
    legend('Roll $\phi$', 'Pitch $\theta$', 'Yaw $\psi$','interpreter','latex', 'location', 'best');
    hold off;
    
    % Plotting control
    figure (3)
    hold on
    box on
    grid on
    set(gca,'FontSize',18,'FontName','Times New Roman');
    plot(Time,Control, 'linewidth', 2);
    xlabel('Time (seconds)');
    ylabel('Control Deflections ($^\circ$)', 'interpreter', 'latex');
    legend('$\delta_t$','$\delta_e$','$\delta_a$','$\delta_r$','interpreter','latex', 'location', 'best');
    hold off;
    
    % Plotting velocities 
    figure (4)
    hold on
    box on
    grid on
    set(gca,'FontSize',18,'FontName','Times New Roman');
    plot(Time,X(1,:),'linewidth',2);
    plot(Time,X(2,:),'linewidth',2);
    plot(Time,X(3,:),'linewidth',2);
    xlabel('Time (seconds)')
    ylabel('Velocity (m/s)')
    legend('$u$', '$v$', '$w$', 'interpreter','latex', 'location', 'best');
    hold off;

    % Plotting body rates 
    figure (5)
    hold on
    box on
    grid on
    set(gca,'FontSize',18,'FontName','Times New Roman');
    plot(Time,X(4,:),'linewidth',2);
    plot(Time,X(5,:),'linewidth',2);
    plot(Time,X(6,:),'linewidth',2);
    xlabel('Time (seconds)')
    ylabel('Body Rates (rad/s)')
    legend('$p$', '$q$', '$r$', 'interpreter','latex', 'location', 'best');
    hold off;
    
    % Plotting xe,ye,ze 
    figure (6)
    hold on
    box on
    grid on
    set(gca,'FontSize',18,'FontName','Times New Roman');
    plot(Time,X(11,:),'linewidth',2);
    plot(Time,X(12,:),'linewidth',2);
    plot(Time,X(13,:),'linewidth',2);
    xlabel('Time (seconds)')
    ylabel('Positions (m)')
    legend('$x_e$','$y_e$', '$z_e$', 'interpreter','latex', 'location', 'best');
    hold off;
    

end