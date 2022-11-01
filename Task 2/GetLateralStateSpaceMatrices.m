% Title: Flight Mechanics Assignment 3 Task 2
% Author: 490412626
% Date: 28/10/22

function [Alat, Blat] = GetLateralStateSpaceMatrices(FlightData, V, theta, h)
    % Import flight data
        % Intertial data
        g   = FlightData.g;
        m   = FlightData.m;
        Ixx = FlightData.Ixx;
        Izz = FlightData.Izz;
        Ixz = FlightData.Ixz;

        % Geometric data
        S   = FlightData.Geo.S;
        b   = FlightData.Geo.b;

        % Side force coefficients
        Cyb = FlightData.Aero.Cyb;
        Cyp = FlightData.Aero.Cyp;
        Cyr = FlightData.Aero.Cyr;
        Cyda = FlightData.Aero.Cyda;
        Cydr = FlightData.Aero.Cydr;

        % N Moment Coefficients
        Cnb = FlightData.Aero.Cnb;
        Cnp = FlightData.Aero.Cnp;
        Cnr = FlightData.Aero.Cnr;
        Cnda = FlightData.Aero.Cnda;
        Cndr = FlightData.Aero.Cndr;

        % L Moment Coefficients
        Clb = FlightData.Aero.Clb;
        Clp = FlightData.Aero.Clp;
        Clr = FlightData.Aero.Clr;
        Clda = FlightData.Aero.Clda;
        Cldr = FlightData.Aero.Cldr;

    % Insert
    % airdensity
    % function
    % Density (kg/m^3) 
    [~, ~, ~, rho] = atmosisa(h);
    
    % Dynamic pressure
    Q = (1/2)*rho*V^2;

    % Aerodynamic derivatives for lateral state space matrix
    Q   = (1/2)*rho*V^2;
    Yv  = (1/(m*V))*Q*S*(Cyb);
    Yp  = (1/m/2/V)*Q*S*b*(Cyp);
    Yr  = (1/m/2/V)*Q*S*b*(Cyr);
    Lv  = (1/(Ixx*V))*Q*S*b*(Clb);
    Lp  = (1/Ixx/2/V)*Q*S*b^2*(Clp);
    Lr  = (1/Ixx/2/V)*Q*S*b^2*(Clr);
    Nv  = (1/(Izz*V))*Q*S*b*(V*Cnb);
    Np  = (1/Izz/2/V)*Q*S*b^2*(Cnp);
    Nr  = (1/Izz/2/V)*Q*S*b^2*(Cnr);
    NTv = 0;
    A1  = Ixz/Ixx;
    B1  = Ixz/Izz;

    % Elements of lateral state space matrix
    A11 = Yv;
    A12 = Yp;
    A13 = Yr - V;
    A14 = g*cos(theta);
    A15 = 0;
    A21 = (Lv + A1*(Nv + NTv))/(1 - A1*B1);
    A22 = (Lp + A1*Np)/(1 - A1*B1);
    A23 = (Lr + A1*Nr)/(1 - A1*B1);
    A24 = 0;
    A25 = 0;
    A31 = ((Nv + NTv) + B1*Lv)/(1 - A1*B1);
    A32 = (Np + B1*Lp)/(1 - A1*B1);
    A33 = (Nr + B1*Lr)/(1 - A1*B1);
    A34 = 0;
    A35 = 0;
    A41 = 0;
    A42 = 1;
    A43 = tan(theta);
    A44 = 0;
    A45 = 0;
    A51 = 0;
    A52 = 0;
    A53 = sec(theta);
    A54 = 0;
    A55 = 0;

    % Lateral state space matrix
    A_lat = [A11 A12 A13 A14 A15;
            A21 A22 A23 A24 A25;
            A31 A32 A33 A34 A35;
            A41 A42 A43 A44 A45;
            A51 A52 A53 A54 A55];  

    % Aerodynamic derivatives for control state space matrix
    Yda = (Q*S*Cyda)/m;
    Ydr = (Q*S*Cydr)/m;
    Lda = (Q*S*b*Clda)/Ixx;
    Ldr = (Q*S*b*Cldr)/Ixx;
    Nda = (Q*S*b*Cnda)/Izz;
    Ndr = (Q*S*b*Cndr)/Izz;

    % Elements of control state space matrix
    B11 = Yda;
    B12 = Ydr;
    B21 = (Lda + A1*Nda)/(1 - A1*B1);
    B22 = (Ldr + A1*Ndr)/(1 - A1*B1);
    B31 = (Nda + B1*Lda)/(1 - A1*B1);
    B32 = (Ndr + B1*Ldr)/(1 - A1*B1);
    B41 = 0;
    B42 = 0;
    B51 = 0;
    B52 = 0;

    % Laterial control state space matrix [B]
    B_lat = [B11 B12;
            B21 B22;
            B31 B32;
            B41 B42;
            B51 B52];
end