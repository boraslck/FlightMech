% angularRates function: generate estimates of alphadot and betadot for force
% generation

function [alphadot, betadot] = AngularRates(Xdot,X)

%% INPUTS (outputs from AeroAngles.m)
% X - state vector
% Xdot - state rate vector
%% OUTPUTS
% alphadot - in rad/s
% betadot - in rad/s

%% FUNCTION

% Extract u v and w
u = X(1);
v = X(2);
w = X(3);

% Compute udot vdot and wdot
udot = Xdot(1);
vdot = Xdot(2);
wdot = Xdot(3);

% Calculate alphadot and betadot
alphadot = (wdot*u - w*udot)/(u^2);
betadot = (vdot*u - v*udot)/(u^2);

end