%% Initialisation

%%
function [Nominal, Secondary] = Initilisation(V,cg)
switch V % Velocity 
    case 100 % 100 knots
        switch CG % CG position
            case 1 % CG position 1

FlightData = aero3560_LoadFlightDataPC9_nominalCG1;

load ICs_PC9_nominalCG1_100Kn_1000ft.mat

            case 2 % CG postion 2
 FlightData = aero3560_LoadFlightDataPC9_CG2;
 
load ICs_PC9_CG2_100Kn_1000ft.mat
  
        end
        
        case 180 % 180 knots
        switch CG
            case 1

FlightData = aero3560_LoadFlightDataPC9_nominalCG1;


load ICs_PC9_nominalCG1_180Kn_1000ft.mat

            case 2
FlightData = aero3560_LoadFlightDataPC9_CG2;
 
load ICs_PC9_CG2_180Kn_1000ft.mat
  
        end
end
               
end