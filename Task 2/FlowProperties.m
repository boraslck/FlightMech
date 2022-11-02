 %% Flow Properties

% This function finds the atmospheric density and the dynamic pressure at a
% given altitude
%%
function [rho,Q] = FlowProperties(X,V)

    % Constants
    T0          = 288.15;           % Sea Level Temperature (K)
    P0          = 101.325;          % Sea Level Pressure (kg/m3)
    hs          = 11e3;             % Troposphere Height (m)
    g           = 9.81;             % Gravitational Acceleration (m/s2)
    L           = 0.0065;           % Lapse Rate
    R           = 0.287058;         % Gas Constant

    % Stratosphere Temperature and Pressure
    Ts          = T0 - L*hs;        
%     Ps          = P0 * (Ts/T0)^(g/L/R);

    % Extract Altitude from the State Vector
    h           = -X(12); 
    
    % Display error in case of altitude exceeding limits
    if h >= 25000
        error('Error in Altitude (FlowProperties)')
    end

    % Determine the density and dynamic pressure depending on the altitude
    if h<= hs
        %Troposphere Properties

        % Temperature
        T       = T0 - L*h;

       % Pressure
        P       = P0 * (T/T0)^5.256;

    else
        % Stratosphere Properties

        % Temperature
        T       = -56.46 + 273.15;

        % Pressure
        P       = 22.65*exp(1.73 - 0.000157*h);

    end

    % Find the Air Density
    rho         = P/R/T; 

    % Find the Dynamic Pressure
    Q           = 0.5*rho*V^2;

end
