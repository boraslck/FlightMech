


function [U_turn, ratio, BodyRates] = roll(FlightData, U_trim, V,X)

    % Unpack aircraft characteristics
    S       = FlightData.Geo.S;
    b       = FlightData.Geo.b;
    Ixx     = FlightData.Inertial.Ixx;
    g       = FlightData.Inertial.g;
    c       = FlightData.Geo.c;
    Cnr     = FlightData.Aero.Cnr;
    Cndr    = FlightData.Aero.Cndr;
    Cnda    = FlightData.Aero.Cnda;
    Clr     = FlightData.Aero.Clr;
    Cldr    = FlightData.Aero.Cldr;
    Clda    = FlightData.Aero.Clda;
    Clp     = FlightData.Aero.Clp;
    Max_da  = FlightData.ControlLimits.Upper(3);
    
    V = 0.514444*V;
    [rho,Q] = FlowProperties(X,V);
    
    
    EoM1 = Q*S*b/Ixx*b/2/V*Clp;
    EoM2 = Q*S*b/Ixx*Clda;
    
    p_dot_max = Q*S*b/Ixx*Clda*Max_da;
    p_max = -2*V/b*Clda/Clp*Max_da;
    
    p = p_max/Max_da*0.1;
    p_dot = p_dot_max/Max_da*0.1;


    fprintf ('%f p + %f da',EoM1,EoM2)
    fprintf('\n')
    
    % Acceleration (g's) - defined by assignment
    nz = sqrt(2);
    
    % Calculate bank angle
    phi = acos(1/nz);
    phi = deg2rad(90);
    
    % Calculate steady heading rate (rad/s)
    psi_dot = (g/V)*tan(phi);
%     psi_dot = 0;
    
    % Calculate yaw rate (rad/s)
    r = psi_dot*cos(phi);
%     r = 0;
    
    % Calculate pitch rate
    q = psi_dot*sin(phi);
%     q = 0
    
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
        % Output controls vector
    U_turn = [U_trim(1); U_trim(2); Max_da; 0];
    
    
    % Approximate ratio of aileron to rudder
    ratio = (Clr*Cndr)/(Cnr*Clda);
    
end