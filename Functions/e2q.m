%% e2q.m
%% Function used to convert euler angles to quaternions

function [quaternion] = e2q(eulers)
    e1 = eulers(1); %euler angle phi
    e2 = eulers(2); %euler angle theta
    e3 = eulers(3); %euler angle psi
    
    %% If statements insuring no negative angles are subbed into the quaternion expressions
    if e1 < 0
        e1 = 360 + e1;
    end
    if e2 < 0
        e1 = 360 + e1;
    end
    if e3 < 0
        e1 = 360 + e1;
    end
    
    %% Quaternion expressions with euler angle inputs
    q0 = cosd(e3/2)*cosd(e2/2)*cosd(e1/2) + sind(e3/2)*sind(e2/2)*sind(e1/2);
    q1 = cosd(e3/2)*cosd(e2/2)*sind(e1/2) - sind(e3/2)*sind(e2/2)*cosd(e1/2);
    q2 = cosd(e3/2)*sind(e2/2)*cosd(e1/2) + sind(e3/2)*cosd(e2/2)*sind(e1/2);
    q3 = -cosd(e3/2)*sind(e2/2)*sind(e1/2) + sind(e3/2)*cosd(e2/2)*cosd(e1/2);

    quaternion(1,1) = q0;
    quaternion(2,1) = q1;
    quaternion(3,1) = q2;
    quaternion(4,1) = q3;
end