function R = Rb2i(phi, theta, psi)
    R = [cos(psi)*cos(theta), sin(phi)*sin(theta)*cos(psi)-sin(psi)*cos(phi), sin(phi)*sin(psi)+sin(theta)*cos(phi)*cos(psi);
         sin(psi)*cos(theta), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), -sin(phi)*cos(psi)+sin(psi)*sin(theta)*cos(phi);
         -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)      
         ];
end

