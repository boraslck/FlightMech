function Cbe = DCM(X)

    % Quaternions from state vector
    q0  = X(7);
    q1  = X(8);
    q2  = X(9);
    q3  = X(10);

    % Transformation Matrices
    l1 = q0^2 + q1^2 - q2^2 - q3^2;
    l2 = 2*(q1*q2 + q0*q3);
    l3 = 2*(q1*q3 - q0*q2);
    m1 = 2*(q1*q2 - q0*q3);
    m2 = q0^2 - q1^2 + q2^2 - q3^2;
    m3 = 2*(q2*q3 + q0*q1);
    n1 = 2*(q0*q2 + q1*q3);
    n2 = 2*(q2*q3 - q0*q1); 
    n3 = q0^2 - q1^2 - q2^2 + q3^2;
    Cbe = [l1 l2 l3; m1 m2 m3; n1 n2 n3];
end