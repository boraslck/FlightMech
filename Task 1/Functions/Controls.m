
% This function finds the controls for a manouver in the simulation

function U_man = Controls(U_Trim, Time, U_filter, T_filter)

U_man = U_Trim;

    if Time <= T_filter(end)

    %         Go through all the timesteps
        for n = 1:length(T_filter)
    %             FIX
            if Time < 1.05*T_filter(n) && Time > 0.95*T_filter(n)
                break
            end

        end
            

            change  = (any(U_filter(1:4,:),2) ~= 0);
            

            % Change the input function accordingly
            U_filter = U_filter(1:4,n);
            U_man(change) = U_filter(change);
            return
        else
    U_man = U_Trim;

    end
end
    



%     
%         