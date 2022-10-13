function x = C_y(x,theta)
    x = [cos(theta), 0, -sin(theta); 0, 1, 0; sin(theta), 0, cos(theta)]*x; 
end