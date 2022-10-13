% angularRates function: generate estimates of alphadot and betadot for force
% generation

function [alphadot, betadot] = AngularRates(X,Xdot)

%% INPUTS (outputs from AeroAngles.m)
% X - state vector
% Xdot - state rate vector
%% OUTPUTS
% alphadot - in rad/s
% betadot - in rad/s

%% FUNCTION

% Extract u v and w
u = X(1,end);
v = X(2,end);
w = X(3,end);

% Compute udot vdot and wdot
udot = Xdot(1,end);
vdot = Xdot(2,end);
wdot = Xdot(3,end);

% Calculate alphadot and betadot
alphadot = (wdot*u - w*udot)/(u^2 + w^2);
betadot = (vdot*u - v*udot)/(u^2 + v^2);

end