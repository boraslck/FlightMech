% Title: AERO3560 Assignment 3 Task A Function (StateRates)
% Author: 490412626
% Date: 12/10/22


function [Xdot, CL, Fa_y] = StateRates(FlightData, X, U, angle_rates)

    % Inputs from State vector
        % Velocity Components
        u   = X(1);
        v   = X(2);
        w   = X(3);
        % Angular Rates
        p   = X(4);
        q   = X(5);
        r   = X(6);
        % Quaternions
        q0  = X(7);
        q1  = X(8);
        q2  = X(9);
        q3  = X(10);

    % Use Previous Functions for all Forces and Moments on the Aircraft
    [Cfa_z, Cfa_x, CL] = WindForces(FlightData, alpha, X, U, V, angle_rates);
    [F_body, M_body] = BodyForces(FlightData, X, U, Cfa_x, Cfa_z, CL, Q, alpha, beta, alpha_dot, beta_dot, V);
    [Fgx, Fgy, Fgz] = Gravity(FlightData, X);
    thrust = PropForces(FlightData, X, U, rho);
    
    % Gather Forces and Moments
        % Forces
        Fa_x = F_body(1);
        Fa_y = F_body(2);
        Fa_z = F_body(3);
        % Moments
        La = M_body(1);
        Ma = M_body(2);
        Na = M_body(3);
    
    % Extract Inertial Parameters Including Aircraft Mass
    m   = FlightData.Inertial.m;          
    Ixx = FlightData.Inertial.Ixx;         
    Iyy = FlightData.Inertial.Iyy;      
    Izz = FlightData.Inertial.Izz ;      
    Ixz = FlightData.Inertial.Ixz;
    
    % Inertial Constants
    c0 = Ixx*Izz - Ixz^2;
    c1 = Izz/c0;
    c2 = Ixz/c0;
    c3 = c2*(Ixx - Iyy + Izz);
    c4 = c1*(Iyy - Izz) - c2*Ixz;
    c5 = 1/Iyy;
    c6 = c5*Ixz;
    c7 = c5*(Izz - Ixx);
    c8 = Ixx/c0;
    c9 = c8*(Ixx - Iyy) + c2*Ixz;
    
    % Accelerations
    udot = r.*v - q.*w + Fgx/m + (Fa_x + thrust)./m;
    vdot = -r.*u + p.*w + Fgy/m + Fa_y./m;
    wdot = q.*u - p.*v + Fgz/m + Fa_z./m;
    
    % Manoeuvring Rates
    pdot = c3.*p.*q + c4.*q.*r + c1.*La + c2.*Na;
    qdot = c7.*p.*r - c6.*(p.^2 - r.^2) + c5.*Ma;
    rdot = c9.*p.*q - c3.*q.*r + c2.*La + c8.*Na;
    
    % Quaternion Rates
    q0dot = -0.5.*(q1.*p + q2.*q + q3.*r);
    q1dot = 0.5.*(q0.*p - q3.*q + q2.*r);
    q2dot = 0.5.*(q3.*p + q0.*q - q1.*r);
    q3dot = -0.5.*(q2.*p - q1.*q - q0.*r);
    
    % Transformation Matrices
    Cbe = DCM(X);
    Ceb = Cbe';

    % Position Rates
    position = Ceb*[u v w]';
    
    % Output State Rates
    Xdot = [udot;vdot;wdot;pdot;qdot;rdot;q0dot;q1dot;q2dot;q3dot;position(1);position(2);position(3)];  
end