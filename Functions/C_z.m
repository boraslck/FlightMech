function x = C_z(x,psi)
    x = [cos(psi), sin(psi), 0; -sin(psi), cos(psi), 0 ; 0 , 0 , 1]*x; 
end