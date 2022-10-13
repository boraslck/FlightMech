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
    l1 = q0^2 + q1^2 - q2^2 - q3^2;
    l2 = 2*(q1*q2 + q0*q3);
    l3 = 2*(q1*q3 - q0*q2);
    m1 = 2*(q1*q2 - q0*q3);
    m2 = q0^2 - q1^2 + q2^2 - q3^2;
    m3 = 2*(q2*q3 + q0*q1);
    n1 = 2*(q0*q2 + q1*q3);
    n2 = 2*(q2*q3 - q0*q1); 
    n3 = q0^2 - q1^2 - q2^2 + q3^2;
    Cbe = [l1 l2 l3; m1 m2 m3; n1 n2 n3];

    % Forces in Body Axis 
    Fg  = Cbe*F;
    Fgx = Fg(1);
    Fgy = Fg(2);
    Fgz = Fg(3);
end