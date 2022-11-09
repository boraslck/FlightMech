% Quick function to estimate Da Dr and Beta from FlightData, phi and
% velocity
function [da dr beta] = estimate_angle(FlightData,phi,V)

% Unpack Flight Data to get coefficients
Cyb = FlightData.Aero.Cyb; Cyda = FlightData.Aero.Cyda; Cydr = FlightData.Aero.Cydr;
Clb = FlightData.Aero.Clb; Clda = FlightData.Aero.Clda; Cldr = FlightData.Aero.Cldr;
Cnb = FlightData.Aero.Cnb; Cnda = FlightData.Aero.Cnda; Cndr = FlightData.Aero.Cndr;

% Get CL as per normal Week 7a pg 6
g = FlightData.Inertial.g; %acceleration due to gravity [m/s^2]
m = FlightData.Inertial.m; %aircraft mass [kg]
S = FlightData.Geo.S; %wing planform area [m^2]
rho = 1.225; %air density [kg/m^3]
CL = (m*g)/((1/2)*rho*(V^2)*S);

phi = deg2rad(phi); %phi or bank angle presumed to be inputed in degrees

% Week 7a pg 7 and 8:
% We can solve using matrices if for the first equation we move each of the
% constants without da, dr or beta to the right. Cyo, Clo, and Clo can be
% ignored since our aircraft doesn't have asymmetric sideforce or moments.

mat = [Cyb Cyda Cydr;...
       Clb Clda Cldr;...
       Cnb Cnda Cndr];

RHS = [CL*phi;...
       0;...
       0]; %right hand side vector
   
soln = mat\RHS; %solution

beta = soln(1);
da = soln(2);
dr = soln(3);

end