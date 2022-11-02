% Title: Flight Mechanics Assignment 3 Task 2
% Author: 490412626
% Date: 28/10/22

% Clear workspace, command window and all figures
clear, clc, close all

% Choose flight condition
V   = 100;  % Velocity [100 or 180]
CG  = 1;    % Centre of mass [1 or 2]

% Initialise Data and get Longitudinal matrices
[A_Lon,B_Lon FlightData X0] = InitialiseMatrix(V,CG);

% Lateral matrices
[A_Lat, B_Lat] = GetLateralStateSpaceMatrices(FlightData, X0); 

% ELiminate x and y positions as they do not impact performance
X0([10, 11]) = [];

% Time variables
dt      = 0.1;          % time step
t_end   = 8;            % total time
time    = 0:dt:t_end;   % time vector

%% Eigen Value Analysis
% Calculate Eigen Values and Vectors
[V_Lon, E_Lon] = eig(A_Lon);
[V_Lat, E_Lat] = eig(A_Lat);

% Calculate Natural Frequencies and Damping Ratios
[Wn_Lon, Z_Lon] = damp(A_Lon);
[Wn_Lat, Z_Lat] = damp(A_Lat);

%% Simulating a response
% Initial State Vector
X(:,1) = X0;

% Choose size of impulse [degrees]
impulse_size = 5;

% Choose time of impulse
t_i = 2;

% Choose which controls are to have a deflection (1 = on, 0 = off)
% In the order of elevator | ailerons | rudder
controls = [0 0 0];

% Check if time of impulse is within the range of time vector
if t_i < time(1) 
    error('This impulse is not within the time period');
elseif t_i > time(length(time))
    error('This impulse is not within the time period');
end

% Loop through vector to detemine controls
for j = 1:length(controls)
    % Control values
    if controls(j) == 1
    control_value(j) = deg2rad(impulse_size);
    elseif controls(j) == 0
    control_value(j) = 0;   
    else
    error('Controls can only be switched on or off indicated by 1 or 0')
    end
end

% Control Vector
U_i = [0; control_value(j)'];

% Loop through time vector
for i = 2:length(time)

    % Is the time period in the impulse time-frame
    if time(i) >= t_i && time(i) < t_i + 0.5

        % Apply impulse
        U = U_i;
    else
        % Otherwise do not apply impulse
        U = [0 0 0 0]';
    end

    % Break state vector into components
    X_Lon = X(1:5,i-1); % Longitudinal
    X_Lat = X(6:10,i-1); % Laterial (the following will keep this convention)

    % Break control into components
    U_Lon = U(1:2);   
    U_Lat = U(3:4);     
        
    % Rates of change
    Lon_dot = A_Lon*X_Lon + B_Lon*U_Lon;
    Lat_dot = A_Lat*X_Lat + B_Lat*U_Lat;
        
    % Euler integration
    X_Lon = X_Lon + Lon_dot*dt;
    X_Lat = X_Lat + Lat_dot*dt;

    % Update state vector
    X(:,i) = [X_Lon;X_Lat];
end





