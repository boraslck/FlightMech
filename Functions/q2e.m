%% q2e.m
%% Function used to convert quaternions to euler angles

function [eulers] = q2e(quats)
    
    q0 = quats(1);
    q1 = quats(2);
    q2 = quats(3);
    q3 = quats(4);

    %% Euler angle expressions with quaternion inputs
    e1 = atan2((q2.*q3 + q0.*q1),(q0.^2 + q3.^2 - (1/2))); %format from equation is "atan2(numerator,denominator)
    e2 = atan2((q0.*q2 - q1.*q3),sqrt((q0.^2 + q1.^2 - (1/2))^2 + (q1.*q2 + q0.*q3).^2));
    e3 = atan2((q1.*q2 + q0.*q3),(q0.^2 + q1.^2 - (1/2)));

    eulers(1,1) = (e1); %phi in radians
    eulers(2,1) = (e2); %theta in radians
    eulers(3,1) = (e3); %psi in radians
end