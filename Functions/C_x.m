function x = C_x(x,phi)
    x = [1, 0, 0; 0, cos(phi), sin(phi); 0, -sin(phi), cos(phi)]*x; 
end