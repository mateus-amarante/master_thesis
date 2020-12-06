function M = Ti2b(phi,theta)
% Angular velocity transformation matrix
% Inertial frame-> Body frame
M=[ 1  0        -sin(theta);
    0  cos(phi)  sin(phi)*cos(theta);
    0 -sin(phi)  cos(phi)*cos(theta)];

end