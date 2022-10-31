% Title: AERO3560 Assignment 3 Task A Function (Integrate)
% Author: 490412626
% Date: 16/10/22

function [X_new] = Integrate(FlightData,X_0,U,dt)

    % Normalise Quaternions
    X_0 = Normalise(X_0);
    
    % State Rate at Step 1
    [Xdot, CL, Cy] = GuessRates(FlightData, X_0, U);
    X_dot_1 = Xdot;
    
    % Incriment at Step 1
    An = X_dot_1*dt;
    
    % State at Step 2
    X_2 = X_0 + An/2;
    
    % Normalise Quaternions
    X_2 = Normalise(X_2);
    
    % State Rate at Step 2
    [Xdot, CL, Cy] = GuessRates(FlightData, X_2, U);
    X_dot_2 = Xdot;
    
    % Incriment at Step 2 
    Bn = X_dot_2*dt;
    
    % State at Step 3
    X_3 = X_0 + Bn/2;
    
    % Normalise Quaternions
    X_3 = Normalise(X_3);
    
    % State Rate at Step 3
    [Xdot, CL, Cy] = GuessRates(FlightData, X_3, U);
    X_dot_3 = Xdot;
    
    % Incriment at Step 3
    Cn = X_dot_3*dt;
    
    % State at Step 4
    X_4 = X_0 + Cn;
    
    % Normalise Quaternions
    X_4 = Normalise(X_4);
    
    % State Rate at Step 4
    [Xdot, CL, Cy] = GuessRates(FlightData, X_4, U);
    X_dot_4 = Xdot;
    
    % Incriment at Step 4
    Dn = X_dot_4*dt;
    
    % New State
    X_new = X_0 + (1/6)*(An + 2*Bn + 2*Cn + Dn);
    
    % Normalise Quaternions
    X_new = Normalise(X_new);
end