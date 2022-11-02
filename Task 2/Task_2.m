% Title: Flight Mechanics Assignment 3 Task 2
% Author: 490412626
% Date: 28/10/22

% Clear workspace, command window and all figures
clear, clc, close all

%<<<<<<< HEAD
%load("aero3560_LoadFlightDataPC9_CG2.mat")

% Load flight data
%FlightData = aero3560_LoadFlightDataPC9_CG2;
%=======

% Load flight data
%[FlightData] = aero3560_LoadFlightDataPC9_CG2();
%>>>>>>> f894f3c675c14c8e6ff7615a90dbc7a0a893d7a2
%FlightData = aero3560_LoadFlightDataPC9_nominalCG1;

% Load input
%load("ICs_PC9_CG2_100Kn_1000ft.mat");
%load("ICs_PC9_CG2_180Kn_1000ft.mat");
%load("ICs_PC9_nominalCG1_100Kn_1000ft.mat");
%load("ICs_PC9_nominalCG1_180Kn_1000ft.mat");

%<<<<<<< HEAD
%=======
%X0([10, 11]) = [];

%>>>>>>> f894f3c675c14c8e6ff7615a90dbc7a0a893d7a2
% Time variables
dt      = 0.1;          % time step
t_end   = 8;            % total time
time    = 0:dt:t_end;   % time vector

%<<<<<<< HEAD
%=======
%[X0 U0 FlightData ] = Initialise(V,CG);

%>>>>>>> f894f3c675c14c8e6ff7615a90dbc7a0a893d7a2
% Input Matrices 
    % Longitudinal
    load("Longitudinal_Matrices_PC9_CG2_100Kn_1000ft.mat");
    %load("Longitudinal_Matrices_PC9_CG2_180Kn_1000ft.mat");
    %load("Longitudinal_Matrices_PC9_nominalCG1_100Kn_1000ft.mat");
    %load("Longitudinal_Matrices_PC9_nominalCG1_100Kn_1000ft.mat");

    % Lateral 
%<<<<<<< HEAD
    [A_lat, B_lat] = GetLateralStateSpaceMatrices(FlightData, X0); 

%% Eigen Value Analysis
% Calculate Eigen Values and Vectors
[V_lon, E_lon] = eig(A_lon);
[V_lat, E_lat] = eig(A_lat);

% Calculate Natural Frequencies and Damping Ratios
[Wn_lon, Z_lon] = damp(A_lon);
[Wn_lat, Z_lat] = damp(A_lat);
%=======
    [A_Lat, B_Lat] = GetLateralStateSpaceMatrices(FlightData, X0); 

%% Eigen Value Analysis
% Calculate Eigen Values and Vectors
[V_Lon, E_Lon] = eig(A_Lon);
[V_Lat, E_Lat] = eig(A_Lat);

% Calculate Natural Frequencies and Damping Ratios
[Wn_Lon, Z_Lon] = damp(A_Lon);
[Wn_Lat, Z_Lat] = damp(A_Lat);
%>>>>>>> f894f3c675c14c8e6ff7615a90dbc7a0a893d7a2

%% Simulating a response
% Initial State Vector
X(:,1) = X0;

% Choose size of impulse [degrees]
impulse_size = 5;

% Choose time of impulse
t_i = 2;

% Choose which controls are to have a deflection (1 = on, 0 = off)
% In the order of elevator | ailerons | rudder
%<<<<<<< HEAD
controls = [0 0 0];
%=======
%controls = [1 1 1];
%>>>>>>> f894f3c675c14c8e6ff7615a90dbc7a0a893d7a2

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
%<<<<<<< HEAD
U_i = [0; control_value(j)'];
%=======
U_i = [0; control_value'];
%>>>>>>> f894f3c675c14c8e6ff7615a90dbc7a0a893d7a2

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
%<<<<<<< HEAD
    X_lon = X(1:5,i-1); % Longitudinal
    X_lat = X(6:10,i-1); % Laterial (the following will keep this convention)

    % Break control into components
    U_lon = U(1:2);   
    U_lat = U(3:4);     
        
    % Rates of change
    Lon_dot = A_lon*X_lon + B_lon*U_lon;
    Lat_dot = A_lat*X_lat + B_lat*U_lat;
        
    % Euler integration
    X_lon = X_lon + Lon_dot*dt;
    X_lat = X_lat + Lat_dot*dt;

    % Update state vector
    X(:,i) = [X_lon;X_lat];
%=======
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
%>>>>>>> f894f3c675c14c8e6ff7615a90dbc7a0a893d7a2

end

% Continue with graphing
%<<<<<<< HEAD
%=======
plot(time,X(1,:))
hold on
grid on
grid minor
plot(time,X(2,:))
plot(time,X(3,:))
%>>>>>>> f894f3c675c14c8e6ff7615a90dbc7a0a893d7a2





