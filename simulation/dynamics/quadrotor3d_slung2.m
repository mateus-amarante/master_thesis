function [qddot] = quadrotor3d_slung(q,qdot,u,physics_p)
%        1
%        |
%   2 -- o -- 4
%        |
%        3
%
%
% q:     [x y z phi theta psi alpha beta]'
% u:     [w1 w2 w4 w4]' (input)
% physics_p: physical parameters
%
% alpha = theta_L
% beta = phi_L
% TODO: - change variable names and check dynamic model.
%       - consider Rphi_L*Rtheta_L instead of Rtheta_L*Rphi_L

% param remapping
M = physics_p.M;
m = physics_p.m;
L = physics_p.L;
g = physics_p.g;
% Ix = physics_p.Ix;
% Iy = physics_p.Iy;
% Iz = physics_p.Iz;
I  = physics_p.I;
Iinv = physics_p.Iinv;
% kt = physics_p.kt;
% km = physics_p.km;
% r = physics_p.r;

% state remapping
% xyz     = q(1:3);
rpy     = q(4:6);
load_rp = q(7:8);
xyz_dot = qdot(1:3);
rpy_dot = qdot(4:6);
load_rp_dot= qdot(7:8);

phiL      = load_rp(1);
thetaL    = load_rp(2);
phiLdot   = load_rp_dot(1);
thetaLdot = load_rp_dot(2);

phi = rpy(1);
theta = rpy(2);
psi = rpy(3);

phidot = rpy_dot(1);
thetadot = rpy_dot(2);
% psidot = rpy_dot(3);

% input remapping
% U = zeros(4,1);
% U(1) = kt*(u'*u);
% wr2 = (u.*u);
% U(2) = kt*r*[0 1 0 -1]*wr2;
% U(3) = kt*r*[-1 0 1 0]*wr2;
% U(4) = km*[ 1 -1 1 -1]*wr2;
U = u;
% 

%% Translation Dynamics
sphiL = sin(phiL); sthetaL = sin(thetaL);
cphiL = cos(phiL); cthetaL = cos(thetaL);

ux = cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
uy = cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
uz = cos(phi)*cos(theta);

% Auxiliar constants
Cf = m*L/(M+m)*(cthetaL^2*phiLdot^2 + thetaLdot^2);
Cb = m/(M*(M + m));

fx = -Cf*sthetaL;
bx = Cb*(sthetaL*cthetaL*(uy*sphiL - uz*cphiL) + ux*(M/m + cthetaL^2));

fy = Cf*sphiL*cthetaL;
by = Cb*(sphiL*cthetaL*(ux*sthetaL + uz*cphiL*cthetaL) + uy*(M/m + 1 - sphiL^2*cthetaL^2));

fz = -Cf*cphiL*cthetaL - g;
bz = Cb*(cphiL*cthetaL*(-ux*sthetaL + uy*sphiL*cthetaL) + uz*(M/m + 1 - cphiL^2*cthetaL^2));

fphiL = 2*tan(thetaL)*phiLdot*thetaLdot;
bphiL = -(uy*cphiL + uz*sphiL)/(M*L*cthetaL);

fthetaL = -sthetaL*cthetaL*phiLdot^2;
bthetaL = (ux*cthetaL + sthetaL*(uy*sphiL - uz*cphiL))/(M*L);


f = [fx fy fz fphiL fthetaL]';
b = [bx by bz bphiL bthetaL]';

linear_acc = f + b*U(1);

%% Angular Dynamics
Tib = Ti2b(phi,theta);
Tbi_dot = Tb2i_dot(phi,theta,phidot,thetadot);
pqr = Tib*rpy_dot;

pqr_dot = Iinv*(U(2:4) + cross(pqr,I*pqr));
rpy_ddot = Tib*(pqr_dot-Tbi_dot*rpy_dot);

%% Output remapping
qddot = [linear_acc(1:3); rpy_ddot; linear_acc(4:5)];

end