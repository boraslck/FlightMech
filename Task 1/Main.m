%% Main

%%% MAIN
clf;
clf reset;
close all;
clear;
clc;

folder = fileparts(which('Main.m')); 
addpath(genpath(folder));

% Main Simulation Script
run Control_GUI
clc

%% Setup

% Choose CG and Velocity
cg_pos      = 1;                % Nominal (1), Secondary (2)
V           = 100;              % Velocity (180), (100)

% Simulation Parameters
dt          = 0.01;             % Delta T
t0          = 0;                % Initial Time
tf          = 400;               % End Time
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
[X_trim U_trim] = Trim (FlightData,X0);
X = X_trim;
U = U_trim;

%% Integrate
% for i = 1:n_pt
%     
% [X_new] = Integrate(FlightData,X_trim,U_trim,dt);
% X(:,i) = X_new;
% X_trim = X_new;
% %U(:,i) = U_new;
% end
fprintf('Load control file to GUI and output to workspace to continue')
pause
clc
T           = t0:dt:tf;         % Points in Time

for i = 2:length(T)
    if T(i) <= 1
        
        [X_new] = Integrate(FlightData,X(:,i-1),U,dt);
        X(:,i) = X_new;
        U(:,i) = U_trim;
        
    else
        
        U_man = Controls(U_trim,T(i),U_filter,T_filter);
        
        [X_new] = Integrate(FlightData,X(:,i-1),U_man,dt);
        X(:,i) = X_new;
        U(:,i) = U_man;
        
    end
    
end

%% Quaternions to Euler Angles
quats = X(7:10,:);
[Eulers] = q2e(quats);

%% Plotting
PlotData(Eulers,U,T,X);






