% Title: AERO3560 Assignment 3 Task A Function (BodyForces)
% Author: 490412626
% Date: 11/10/22

function [F_body, M_body] = BodyForces(FlightData, X, U, Cfa_x, Cfa_z, CL, Q, alpha, beta, alpha_dot, beta_dot, V)

    % Aerodynamic Parameters
        % Side Force Coefficients
        Clb     = FlightData.Aero.Clb;
        Clbd    = FlightData.Aero.Clbd;
        Clp     = FlightData.Aero.Clp;
        Clr     = FlightData.Aero.Clr;
        Clda    = FlightData.Aero.Clda;
        Cldr    = FlightData.Aero.Cldr;
        % M Moment Coefficients
        Cmo     = FlightData.Aero.Cmo;
        Cma     = FlightData.Aero.Cma;
        Cmq     = FlightData.Aero.Cmq;
        Cmad    = FlightData.Aero.Cmad;
        Cmde    = FlightData.Aero.Cmde;
        % N Moment Coefficients
        Cnb     = FlightData.Aero.Cnb;
        Cnbd    = FlightData.Aero.Cnbd;
        Cnp     = FlightData.Aero.Cnp;
        Cnr     = FlightData.Aero.Cnr;
        Cnda    = FlightData.Aero.Cnda;
        Cndr    = FlightData.Aero.Cndr;
        % L Moment Coefficients
        Cyb     = FlightData.Aero.Cyb;
        Cybd    = FlightData.Aero.Cybd;
        Cyp     = FlightData.Aero.Cyp;
        Cyr     = FlightData.Aero.Cyr;
        Cyda    = FlightData.Aero.Cyda;
        Cydr    = FlightData.Aero.Cydr;

    % Geometric Parameters
    S = FlightData.Geo.S;
    c = FlightData.Geo.c;
    b = FlightData.Geo.b;
    
    % Input from State Vector
        % Anglular Rates
        p   = X(4);
        q   = X(5);
        r   = X(6);
        % Quaternions
        q0  = X(7);
        q1  = X(8);
        q2  = X(9);
        q3  = X(10);

    % Relavent Control Inputs
    delta_e = U(2);
    delta_a = U(3);
    delta_r = U(4);

    % Non-dimensionalise Angular Rates
    p_hat           = (p*c)/(2*V);
    q_hat           = (q*c)/(2*V);
    r_hat           = (r*c)/(2*V);
    alpha_dot_hat   = (alpha_dot*c)/(2*V);
    beta_dot_hat    = (beta_dot*c)/(2*V);

    % Forces on the Aircraft from Wind
    Fa_x = Q*Cfa_x*S;
    Fa_z = Q*Cfa_z*S;
    
    % Side Force Coefficient
    Cy = Cyb*beta + Cybd*beta_dot_hat + Cyp*p_hat + Cyr*r_hat + Cyda*delta_a + Cydr*delta_r;

    % Side Force
    Fa_y = Q*Cy*S;

    % Moment Coefficients
    Cl = Clb*beta + Clp*p_hat + Clr*r_hat + Clbd*beta_dot_hat + Clda*delta_a + Cldr*delta_r;
    Cm = Cmo + Cma*alpha + Cmq*q_hat + Cmad*alpha_dot_hat + Cmde*delta_e;
    Cn = Cnb*beta + Cnp*p_hat + Cnr*r_hat + Cnbd*beta_dot_hat + Cnda*delta_a + Cndr*delta_r;

    % Moments
    La = Q*Cl*S*b;
    Ma = Q*Cm*S*c;
    Na = Q*Cn*S*b;

    % Rotate from Euler Angles to Body Axis
    cy = C_y(alpha);
    cz = C_z(-beta);
   
    % Output Vectors of Forces and Moments in the Body Axis
    F_body = cy*cz*[-Fa_x Fa_y Fa_z]';
    M_body = cy*cz*[La Ma Na]';
end










    