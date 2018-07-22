function M = Tb2i(phi,theta)
% Angular velocity transformation matrix
% Body frame-> Inertial frame
M=[ 1  sin(phi)*tan(theta)   cos(phi)*tan(theta);
    0  cos(phi)             -sin(theta);
    0  sin(phi)/cos(theta)   cos(phi)/cos(theta)];

end