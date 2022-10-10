%% AeroAngles

% This function converts the body vectors to flight vector components
%%
function [V, alpha, beta] = AeroAngles(X)

    % Extract State Vector Velocities
    u           = X(1);     
    v           = X(2);
    w           = X(3);
    
    % Calculate the Angles
    V           = norm([u,v,w]);    % Total Velocity (m/s)
    beta        = asin(V/W);        % Sideslip Angle (rad)
    alpha       = atan(w./u);       % Angle of Attack (rad)
    
end