% Title: Flight Mechanics Assignment 3 Task 2
% Author: 490412626
% Date: 28/10/22

% Clear workspace, command window and all figures
clear, clc, close all

% Input Matrices 
    A_long = [];
    % Lateral uses dv, dp, dr, dphi, dpsy
    A_lat = [];

% Calculate Eigen Values and Vectors
[V_long, E_long] = eig(A_long);
[V_lat, E_lat] = eig(A_lat);

% Calculate Natural Frequencies and Damping Ratios
[Wn_long, Z_long] = damp(A_long);
[Wn_lat, Z_lat] = damp(A_lat);