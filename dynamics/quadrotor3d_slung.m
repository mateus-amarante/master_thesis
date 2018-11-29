function [qddot] = quadrotor3d_slung(q,qdot,u,physics_p)
% FIXME: NOT WORKING

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

%% Translation Dynamics
sphiL = sin(phiL); sthetaL = sin(thetaL);
cphiL = cos(phiL); cthetaL = cos(thetaL);

m15 = -m*L*cthetaL;       m51 = m15;

m24 =  m*L*cphiL*cthetaL; m42 = m24;
m25 = -m*L*sphiL*sthetaL; m52 = m25;

m34 =  m*L*sphiL*cthetaL; m43 = m34;
m35 =  m*L*cphiL*sthetaL; m53 = m35;

Mq = [
    M+m  0    0    0                m15;
    0    M+m  0    m24              m25;
    0    0    M+m  m34              m35;
    0    m42  m43  m*L^2*cthetaL^2  0;
    m51  m52  m53    0,             m*L^2];


c15 =  m*L*sthetaL*thetaLdot;
c24 = -m*L*(sphiL*cthetaL*phiLdot   + cphiL*sthetaL*thetaLdot);
c25 = -m*L*(sphiL*cthetaL*thetaLdot + cphiL*sthetaL*phiLdot);
c34 =  m*L*(cphiL*cthetaL*phiLdot   - sphiL*sthetaL*thetaLdot);
c35 =  m*L*(cphiL*cthetaL*thetaLdot - sphiL*sthetaL*phiLdot);
c44 = -m*L^2*sthetaL*cthetaL*thetaLdot;
c45 = -m*L^2*sthetaL*cthetaL*phiLdot;
c54 = -c45;

Cq = [
    0 0 0 0   c15;
    0 0 0 c24 c25;
    0 0 0 c34 c35;
    0 0 0 c44 c45;
    0 0 0 c54 0];

Gq = [0; 0; (M+m)*g; m*g*L*sphiL*cthetaL; m*g*L*cphiL*sthetaL];

ux = cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
uy = cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
uz = cos(phi)*cos(theta);

bq = [ux uy uz 0 0]';


linear_acc = Mq\(-Cq*[xyz_dot;load_rp_dot] -Gq + bq*U(1));


% Angular Dynamics
Tib = Ti2b(phi,theta);
Tbi_dot = Tb2i_dot(phi,theta,phidot,thetadot);
pqr = Tib*rpy_dot;

pqr_dot = Iinv*(U(2:4) + cross(pqr,I*pqr));
rpy_ddot = Tib*(pqr_dot-Tbi_dot*rpy_dot);

% Output remapping
qddot = [linear_acc(1:3); pqr_dot; linear_acc(4:5)];

end