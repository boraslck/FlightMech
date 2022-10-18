%% Function 

%% 
function [Xdot, CL, Cy] = GuessRates(FlightData, X, U)

alpha_zero = 0;
beta_zero = 0; 

% Iteration parameters
error = 1;
toler = 1e-9;
IterLim = 100;
IterCount = 0;

while error > toler
   
    angle_rates = [alpha_zero, beta_zero];
    [Xdot CL Cy] = StateRates (FlightData, X, U, angle_rates);
    
    [alpha_dot, beta_dot] = AngularRates(Xdot,X);
    
    error_alp = abs((alpha_dot - alpha_zero)/ alpha_zero);
    error_bet = abs((alpha_dot - alpha_zero)/ alpha_zero);
    error_abs = max([error_alp error_bet]);
    
    alpha_zero = alpha_dot;
    beta_zero = beta_dot;
    
    if IterCount > IterLim
        fprintf('Guess Rates Iteration Limit Error')
        break
    end
    
    IterCount = IterCount + 1;
    
    
    
end
    
    
end