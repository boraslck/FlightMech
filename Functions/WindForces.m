%% WindForces

% This function evaluates the non-dimensional coefficients for aerodynamic
% forces (lift and drag coefficients)

%%
function [Cf_z, Cd, CL] = WindForces(FlightData, X, U, V, angle_rates, alpha)

% Get Flight Data
CL0         = FlightData.Aero.CLo;      % (nondim)
CLa         = FlightData.Aero.CLa;      % (\rad)
CLq         = FlightData.Aero.CLq;      % (nondim)  
CLde        = FlightData.Aero.CLde;     % (\rad)  
CLad        = FlightData.Aero.CLad;     % (\rad)
Cd0         = FlightData.Aero.Cdo;      % (nondim)
k           = FlightData.Aero.k;        % (nondim)
c           = FlightData.Geo.c;         % m

% Extract the angle rates of change
alpha_dot       = angle_rates(1);

% Extract pitch rate from Body Rates from State Vector
q               = X(5);

% Find the Elevator Control from Control Vector
delta_e         = U(2);

% Find the non-dimensional angular rates
q_hat           = q*c/2/V;
alpha_dot_hat   = alpha_dot*c/2/V;

% Find the Lift and Drag Coefficients
CL = -CL0 - CLa*alpha - CLq * q_hat  - CLde*delta_e - CLad * alpha_dot_hat;
Cd = Cd0 + k*CL^2;

Cf_z = CL;
end
