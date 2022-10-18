%% Main

%%% MAIN
clf;
clf reset;
close all;
clear;
clc;

% Main Simulation Script
run Control_GUI
clc

%% Setup

% Choose CG and Velocity
cg_pos      = 1;                % Nominal (1), Secondary (2)
V           = 100;              % Velocity (180), (100)

% Simulation Parameters
dt          = 0.01;             % Delta T
t0          = dt;               % Initial Time
tf          = 500;              % End Time
n_pt        = tf/dt;            % Number of Intervals

% Initialise Data for faster process time
X           = zeros(13,n_pt);   % State Vector
U           = zeros(4,n_pt);    % Control Vector
T           = zeros(1,n_pt);    % Time Vector

%% Initialise
% Specify the Flight Data
[X0_init U0_init FlightData] = Initialise(V,cg_pos);

% Convert to quaternions
quats_init  = e2q(X0_init(7:9));
X0          = [X0_init(1:6); quats_init; X0_init(10:12)];
U0          = U0_init;

%% Trim Routine

[X_trim U_Trim] = Trim (FlightData,X0);











