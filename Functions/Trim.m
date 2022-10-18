%% Trim

% This function solves for the forces on the aircraft body axes

%% Function

function [X_trim, U_trim] = Trim(FlightData, X0)

    % Extract Aircraft Flight Data
    m                   = FlightData.Inertial.m;    % mass(kg)
    g                   = FlightData.Inertial.g;    % Gravity (m/s2)
    S                   = FlightData.Geo.S;         % Wing Area (m2)
    CLa                 = FlightData.Aero.CLa;      % (/rad)
    CL0                 = FlightData.Aero.CLo;      % (nondim)

    % AeroAngles
    % Get the aero angles using the current body velocities
    [V,alpha,beta]    = AeroAngles(X0);

    % FlowProperties
    % Find the flow conditions
    [rho,Q]             = FlowProperties (X0,V);

    % Find an estimate for the lift coefficient
    CL                  = m*g/Q/S; 
    alpha_0             = (CL-CL0)/CLa;
    U0                  = [0.5; 0; 0; 0];

    % Set up Jacobian and perturbation and convergence limiters
    % Set the rate of change to be zero for state vector
    tr                  = [1 3 5];
    Jacobian            = zeros(3);

    delta               = [1e-6 ; 1e-6 ; 1e-6];     % Increment
    x_bar               = [alpha_0 ; U0(1) ; U0(2)];

    convergence         = 0;                        % Convergence (false)  
    Lim                 = 250;                      % Iteration Limit
    toler               = 1e-9;                     % TOlerance
    count               = 1;                        % Iteration Count

    % Repeat iterations until convergence
    while ~convergence

        % Extract Attitude and Normalize
        att_eul         = q2e(X0(7:10));
        att_eul(2)      = x_bar(1);
        quats           = e2q(att_eul);
        X0(7:10)        = quats/norm(quats);
        


        % State Rates 
        X_dot           = GuessRates(FlightData,X0,U0);

        f_bar           = X_dot(tr);

        % JACOBIAN perturbations
            for n=1:3

                % Initialise the variables for trimming
                NextX = X0;
                NextU = U0;

                % Angle of attack perturbation
                if n ==1
                    % alpha and u
                    NextX(1) = V*cos(x_bar(n) + delta(n));
                    NextX(3) = V*sin(x_bar(n) +delta(n));

                    % Control perturbatiosns
                else
                    % dt and de
                   NextU(n-1) = x_bar(n) + delta(n);

                end

                % Find the state rates for the new state and control
                NextX_dot = GuessRates(FlightData, NextX, NextU);

                % Locate the corresponding matrix
                Jacobian(:,n) = (NextX_dot(tr) - X_dot(tr))./delta;
            end

            % Renew x_bar and find error
            x_bar_next = x_bar - Jacobian\f_bar;
            err = abs((x_bar_next - x_bar)./ delta);

            % Check for convergence
            if max(err)<= toler
                break
                fprint('Trim has not converged')
            end
   
            % Check for iteration limit
            if count>= Lim 
                break
            end

            % Continue if converged
            x_bar = x_bar_next;

            % State vectors are updated
            X0(1) = V*cos(x_bar(1));
            X0(3) = V*sin(x_bar(1));

            % Update U vector
            U0(1) = x_bar(2);
            U0(2) = x_bar(3);

            % Increment
            count  = count +1;
    end

    % Trim the vectors to final state
            X_trim = X0;
            U_trim = U0;
end