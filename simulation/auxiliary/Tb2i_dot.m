function M = Tb2i_dot(phi,theta,phidot,thetadot)
% 1st derivative of angular velocity transformation matrix
% Body frame-> Inertial frame

ctheta = cos(theta);
sphi = sin(phi);
cphi = cos(phi);
tgtheta = tan(theta);

M=[ 0  sphi*thetadot/ctheta^2+cphi*tgtheta*phidot  -sphi*tgtheta*phidot+cphi*thetadot/ctheta^2;
    0 -sphi*phidot                                 -cphi*phidot;
    0  (sphi*tgtheta*thetadot+cphi*phidot)/ctheta   (-sphi*phidot+cphi*tgtheta*thetadot)/ctheta];

end