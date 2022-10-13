% Title: AERO3560 Assignment 3 Task A Function (PropForces)
% Author: 490412626
% Date: 11/10/22

function thrust = PropForces(FlightData, X, U, rho)

    % Propeller Data
    PmaxSL = FlightData.Prop.P_max;
    eta = FlightData.Prop.eta;
    
    % Velocity in X-Direction from State Vector
    u = X(1);
    
    % Thrust Setting from Control Vector
    delta_t = U(1);

    % Air Density Ratio
    rho_SL = 1.2256;
    sigma = rho/rho_SL;

    % Available Thrust
    Pmax = PmaxSL*(1.1324*sigma - 0.1324);

    % Thrust Produced
    thrust = (Pmax*eta*delta_t)/u;
end