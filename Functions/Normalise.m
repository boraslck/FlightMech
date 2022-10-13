% Title: AERO3560 Assignment 3 Task A Function (Normalise)
% Author: 490412626
% Date: 13/10/22

function X = Normalise(X)

    % Initial Quaternions from State Vector 
    q0 = X(7);
    q1 = X(8);
    q2 = X(9);
    q3 = X(10);

    % Vector of Quaternions
    quat = [q0 q1 q2 q3]';

    % Normalise Quaternions
    X(7:10) = quat/(sqrt(q0^2 + q1^2 + q2^2 + q3^2));
end