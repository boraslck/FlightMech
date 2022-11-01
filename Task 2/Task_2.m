% Title: Flight Mechanics Assignment 3 Task 2
% Author: 490412626
% Date: 28/10/22

% Clear workspace, command window and all figures
clear, clc, close all

% Must load in V, theta, h, X_0, time vector
dt      = 0.1;          % time step
t_end   = 8;            % total time
time    = 0:dt:t_end;   % time vector

% Choose Flight Conditions
V       = 100;
CG      = 1;
[A_Lon B_Lon FlightData ] = InitialiseMatrix(V,CG);
% Input Matrices 
    % Longitudinal
    % Lateral 
    [Alat, Blat] = GetLateralStateSpaceMatrices(FlightData, V, theta, h); 

%% Eigen Value Analysis
% Calculate Eigen Values and Vectors
[V_long, E_long] = eig(A_long);
[V_lat, E_lat] = eig(A_lat);

% Calculate Natural Frequencies and Damping Ratios
[Wn_long, Z_long] = damp(A_long);
[Wn_lat, Z_lat] = damp(A_lat);

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
    X_long = X(1:5,i-1); % Longitudinal
    X_lat = X(6:10,i-1); % Laterial (the following will keep this convention)

    % Break control into components
    U_long = U(1:2);   
    U_lat = U(3:4);     
        
    % Rates of change
    Long_dot = A_long*X_long + B_long*U_long;
    Lat_dot = A_lat*X_lat + B_lat*U_lat;
        
    % Euler integration
    X_long = X_long + Long_dot*dt;
    X_lat = X_lat + Lat_dot*dt;

    % Update state vector
    X(:,i) = [X_long;X_lat];

end

% Continue with graphing





