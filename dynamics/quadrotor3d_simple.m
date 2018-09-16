function [q_ddot] = quadrotor3d(q,qdot,u,physics_p)
%        1
%        |
%   2 -- o -- 4
%        |
%        3
%
%
% q:     [x y z phi theta psi]'
% u:     [w1 w2 w4 w4]' (input)
% physics_p: physical parameters

% param remapping
M = physics_p.M;
g = physics_p.g;
Ix = physics_p.Ix;
Iy = physics_p.Iy;
Iz = physics_p.Iz;
% I  = physics_p.I;
% Iinv = physics_p.Iinv;
kt = physics_p.kt;
km = physics_p.km;
r = physics_p.r;

% state remapping
% xyz     = q(1:3);
rpy     = q(4:6);
% xyz_dot = qdot(1:3);
rpy_dot = qdot(4:6);

phi = rpy(1);
theta = rpy(2);
psi = rpy(3);

phidot = rpy_dot(1);
thetadot = rpy_dot(2);

% input remapping
U = zeros(4,1);
U(1) = kt*(u'*u);
wr2 = (u.*u);
U(2) = kt*r*[0 1 0 -1]*wr2;
U(3) = kt*r*[-1 0 1 0]*wr2;
U(4) = km*[ 1 -1 1 -1]*wr2;

% dynamics
xddot = 1/M*(cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi))*U(1);
yddot = 1/M*(cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi))*U(1);
zddot = -g + 1/M*cos(phi)*cos(theta)*U(1);

phiddot   = (Iy-Iz)/Ix*thetadot*psidot + U(2)/Ix;
thetaddot = (Iz-Ix)/Iy*phidot*psidot   + U(3)/Iy;
psiddot   = (Ix-Iy)/Iz*phidot*thetadot + U(4)/Iz;

xyz_ddot = [xddot; yddot; zddot];
rpy_ddot = [phiddot; thetaddot; psiddot];

% output remapping
q_ddot = [xyz_ddot; rpy_ddot];

end