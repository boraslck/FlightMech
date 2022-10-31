% Title: AERO3560 Assignment 3 Task A Function (Gravity)
% Author: 490412626
% Date: 12/10/22

function [Fgx, Fgy, Fgz] = Gravity(FlightData, X)

    % Extract Value from Flight Data
    g = FlightData.Inertial.g;
    m = FlightData.Inertial.m;

    % Weight Force
    W = m*g;

    % Quaternions from State Vector
    q0  = X(7);
    q1  = X(8);
    q2  = X(9);
    q3  = X(10);
    
    % Weight Force Vector in Earth Axis
    F = [0 0 W]';
    
    % Transformation Matrix
    Cbe = DCM(X);

    % Forces in Body Axis 
    Fg  = Cbe*F;
    Fgx = Fg(1);
    Fgy = Fg(2);
    Fgz = Fg(3);
end