% AERO3560 - Flight Mechanics 1 - Assignment 3 2018
% Author SID: 460306678, 460369684, 460373315, 460369189
% Function Name: simulate
%
% Function Description:
%   Simulates the flight path on aircraft model. Calls 'c130' function
%   produced by 'C. Green': 
%   https://au.mathworks.com/matlabcentral/fileexchange/47967-draw-a-3d-...
%   airplane?focused=6019548&tab=function
%
% Inputs:
%   X:      Vector containing the aircraft state. The order is:
%               - u   = X(1)    (m/s)
%               - v   = X(2)    (m/s)
%               - w   = X(3)    (m/s)
%               - p   = X(4)    (rad/s)
%               - q   = X(5)    (rad/s)
%               - r   = X(6)    (rad/s)
%               - q0  = X(7)    -
%               - q1  = X(8)    -
%               - q2  = X(9)    -
%               - q3  = X(10)   -
%               - x   = X(11)   (m)
%               - y   = X(12)   (m)
%               - z   = X(13)   (m)
%
% Outputs: none
%
% Other m-files required: none
%
% Subfunctions: none
%
% MAT-files required: none
%
% TODO: none

function simulate(X)

% Create Ground Surface
cdata = imread('aerial.jpg');


% Set visual properties for plot
props.AmbientStrength = 0.1;
props.DiffuseStrength = 1;
props.SpecularColorReflectance = .5;
props.SpecularExponent = 20;
props.SpecularStrength = 1;
props.FaceColor= 'texture';
props.EdgeColor = 'none';
props.FaceLighting = 'phong';
props.Cdata = cdata;


figure(6)
[x, y] = meshgrid(min([X(11,:),X(12,:)]):10:max([X(11,:),X(12,:)]));
z = zeros(size(x,1));
surf(x,y,z, props);
axis([-5000 5000 -5000 5000 0 -min(X(13,:))])
axis equal
axis off
set(gcf, 'Color', [1 1 1]);
set(gca, 'Color', [1 1 1]);
hold on

% Set view angle
view(155,22)

x = X(12,:) + 2500;
y = X(11,:);
z = -X(13,:);
euler = rad2deg(quat2euler(X(7:10,:)));


% Create orbit
figure(6)
flightpath = animatedline('LineWidth',1.5,'Color','b');

% Plotting frequency
plotFreq = 100;

% Loop through time vector
for k = 1:length(x)
    
    % Break if crash
    if z(k) <= 0
        plot3(x(k),y(k),z(k),'or','MarkerSize',10,'MarkerFaceColor','r');
        delete(plane);
        return
    end
    
    % Add point to the curve
    addpoints(flightpath,x(k),y(k),z(k));
    
    
    if mod(k,plotFreq) == 0
               
        % Add aircraft        
        plane = c130(x(k),y(k),z(k),'color','r','scale',10,'roll', ...
            euler(1,k), 'pitch', euler(2,k), 'yaw', -euler(3,k));
                      
        % Pause 
        pause(0.0001)
        
        if k < length(x) - plotFreq
            % Remove plane
            delete(plane)
        end
    end
end

% Delete aircraft
delete(plane)

% Permanently plot plane in last simulated position/orientation
plane = c130(x(k),y(k),z(k),'color','r','scale',10,'roll',euler(1,k), ...
    'pitch',euler(2,k),'yaw',-euler(3,k));
end