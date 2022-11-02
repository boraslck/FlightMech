


function [U_turn, ratio, BodyRates] = st_turn(Params, U_trim, V)

    % Unpack aircraft characteristics
    g       = Params.Inertial.g;
    c       = Params.Geo.c;
    Cnr     = Params.Aero.Cnr;
    Cndr    = Params.Aero.Cndr;
    Cnda    = Params.Aero.Cnda;
    Clr     = Params.Aero.Clr;
    Cldr    = Params.Aero.Cldr;
    Clda    = Params.Aero.Clda;
    
    V = 0.514444*V;

    % Acceleration (g's) - defined by assignment
    nz = sqrt(2);
    
    % Calculate bank angle
    phi = acos(1/nz);
%     phi = deg2rad(45);
    
    % Calculate steady heading rate (rad/s)
    psi_dot = (g/V)*tan(phi);
    
    % Calculate yaw rate (rad/s)
    r = psi_dot*cos(phi);
    
    % Calculate pitch rate
    q = psi_dot*sin(phi);
    
    % Store body rates
    BodyRates.r = r;
    BodyRates.q = q;
    
    % Non-dimensionalise yaw rate
    r_hat = (r*c)/(2*V);
    
    % Estimate aileron deflection (rad)
    delta_a = ((Cldr*Cnr - Cndr*Clr)/(Cndr*Clda - Cldr*Cnda))*r_hat;     
    
    % Estimate rudder deflection
    delta_r = ((Cnda*Clr - Clda*Cnr)/(Cndr*Clda - Cldr*Cnda))*r_hat;
    
    % Output controls vector
    U_turn = [U_trim(1); U_trim(2); delta_a; delta_r];
    
    % Approximate ratio of aileron to rudder
    ratio = (Clr*Cndr)/(Cnr*Clda);
    
end