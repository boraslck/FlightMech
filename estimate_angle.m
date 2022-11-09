% Quick function to find Da Dr and Beta
function [da dr beta] = estimate_angle(FlightData,phi)

% Unpack Flight Data to get coefficients
Cyb = FlightData.Aero.Cyb; Cyda = FlightData.Aero.Cyda; Cydr = FlightData.Aero.Cydr;
Clb = FlightData.Aero.Clb; Clda = FlightData.Aero.Clda; Cldr = FlightData.Aero.Cldr;
Cnb = FlightData.Aero.Cnb; Cnda = FlightData.Aero.Cnda; Cndr = FlightData.Aero.Cndr;

% Get CL from lift equation

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