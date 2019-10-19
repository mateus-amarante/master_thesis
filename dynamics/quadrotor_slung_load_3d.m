function [qdot] = quadrotor_slung_load_3d(q,u,physics_p)
%
%        1
%        |
%   2 -- o -- 4
%        |
%        3
%
%
% q:     [x y z phi theta psi phiL thetaL]'
% u:     [Fz_b tau_x tau_y tau_z]' (input)
% physics_p: physical parameters
%


%% PARAM REMAPPING
M = physics_p.M;
m = physics_p.m;
l = physics_p.l;
g = physics_p.g;
% Ix = physics_p.Ix;
% Iy = physics_p.Iy;
% Iz = physics_p.Iz;
I  = physics_p.I;
Iinv = physics_p.Iinv;
% kt = physics_p.kt;
% km = physics_p.km;
% r = physics_p.r;

% Drag coefficients
cx = physics_p.cx;
cy = physics_p.cy;
cz = physics_p.cz;
cL = physics_p.cL;

%% STATE REMAPPING

% Position
% xyz     = q(1:3);
rpy     = q(4:6);
load_rp = q(7:8);

% Velocity
xyz_dot = q(9:11);
% pqr = q(12:14);
rpy_dot = q(12:14);
load_rp_dot= q(15:16);

% Load angular position and velocity
phiL      = load_rp(1);
thetaL    = load_rp(2);
phiLdot   = load_rp_dot(1);
thetaLdot = load_rp_dot(2);

% Quadrotor orientation and angular velocity
phi = rpy(1);
theta = rpy(2);
psi = rpy(3);

pqr = Ti2b(phi, theta)*rpy_dot;
% rpy_dot = Tb2i(phi, theta)*pqr;
phidot = rpy_dot(1);
thetadot = rpy_dot(2);
psidot = rpy_dot(3);

%% INPUT REMAPPING
% U = zeros(4,1);
% U(1) = kt*(u'*u);
% wr2 = (u.*u);
% U(2) = kt*r*[0 1 0 -1]*wr2;
% U(3) = kt*r*[-1 0 1 0]*wr2;
% U(4) = km*[ 1 -1 1 -1]*wr2;
U = u(:);

%% COMPUTE ACCELERATIONS
% Helper variables
sphiL = sin(phiL); sthetaL = sin(thetaL);
cphiL = cos(phiL); cthetaL = cos(thetaL);

% Inertial matrix
% m15 = -m*l*cthetaL;       m51 = m15;

% m24 =  m*l*cphiL*cthetaL; m42 = m24;
% m25 = -m*l*sphiL*sthetaL; m52 = m25;

% m34 =  m*l*sphiL*cthetaL; m43 = m34;
% m35 =  m*l*cphiL*sthetaL; m53 = m35;

% Mq = [
%     M+m  0    0    0                m15;
%     0    M+m  0    m24              m25;
%     0    0    M+m  m34              m35;
%     0    m42  m43  m*l^2*cthetaL^2  0;
%     m51  m52  m53  0                m*l^2];

% Inverse of Inertial matrix
m11 = (M + m*cthetaL^2)/(M*(M + m));
m22 = (M + m*(1 - sphiL^2*cthetaL^2))/(M*(M + m));
m33 = (M + m*(1 - cphiL^2*cthetaL^2))/(M*(M + m));

m12 =  m*sphiL*sthetaL*cthetaL/(M*(M + m)); m21 = m12;
m13 = -m*cphiL*sthetaL*cthetaL/(M*(M + m)); m31 = m13;
m23 =  m*sphiL*cphiL*cthetaL^2/(M*(M + m)); m32 = m23;

m44 = (M + m)/(M*m*l^2*cthetaL^2);
m55 = (M + m)/(M*m*l^2);

m15 = cthetaL/(M*l);        m51 = m15;
m24 = -cphiL/(M*l*cthetaL); m42 = m24;
m25 = sphiL*sthetaL/(M*l);  m52 = m25;
m34 = -sphiL/(M*l*cthetaL); m43 = m34;
m35 = -cphiL*sthetaL/(M*l); m53 = m35;

invMq = [
    m11  m12  m13    0  m15;
    m21  m22  m23  m24  m25;
    m31  m32  m33  m34  m35;
      0  m42  m43  m44    0;
    m51  m52  m53    0  m55];

% Coriolis/Centrifugal forces matrix
c15 =  m*l*sthetaL*thetaLdot;
c24 = -m*l*(sphiL*cthetaL*phiLdot   + cphiL*sthetaL*thetaLdot);
c25 = -m*l*(sphiL*cthetaL*thetaLdot + cphiL*sthetaL*phiLdot);
c34 =  m*l*(cphiL*cthetaL*phiLdot   - sphiL*sthetaL*thetaLdot);
c35 =  m*l*(cphiL*cthetaL*thetaLdot - sphiL*sthetaL*phiLdot);
c44 = -m*l^2*sthetaL*cthetaL*thetaLdot;
c45 = -m*l^2*sthetaL*cthetaL*phiLdot;
c54 = -c45;

Cq = [
    0 0 0 0   c15;
    0 0 0 c24 c25;
    0 0 0 c34 c35;
    0 0 0 c44 c45;
    0 0 0 c54 0];

% Gravitational matrix
Gq = [0; 0; (M+m)*g; m*g*l*sphiL*cthetaL; m*g*l*cphiL*sthetaL];

% Input matrix
ux = cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
uy = cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
uz = cos(phi)*cos(theta);

Bq = [ux uy uz 0 0]';

% Drag matrix
p11 = -(cx + cL);
p22 = -(cy + cL);
p33 = -(cz + cL);
p44 = -cL*l^2*cthetaL^2;
p55 = -cL*l^2;

p15 =  cL*l*cthetaL;       p51 = p15;
p24 = -cL*l*cphiL*cthetaL; p42 = p24;
p25 =  cL*l*sphiL*sthetaL; p52 = p25;
p34 = -cL*l*sphiL*cthetaL; p43 = p34;
p35 = -cL*l*cphiL*sthetaL; p53 = p35;

Pq = [
    p11    0    0    0  p15;
    0    p22    0  p24  p25;
    0      0  p33  p34  p35;
    0    p42  p43  p44    0;
    p51  p52  p53  0    p55
];

% Translational dynamic Equation
transl_acc = invMq*((Pq - Cq)*[xyz_dot;load_rp_dot] -Gq + Bq*U(1));
xyz_ddot = transl_acc(1:3);
phiLthetaL_ddot = transl_acc(4:5);

% Angular Dynamics
pqr_dot = Iinv*(U(2:4) - cross(pqr,I*pqr));  % missing gyroscopic effect of the rotors
rpy_ddot = Tb2i(phi,theta)*pqr_dot + Tb2i_dot(phi,theta,phidot,thetadot)*pqr;
% OUTPUT REMAPPING
% qdot = [xyz_dot; rpy_dot; phiLdot; thetaLdot; xyz_ddot; pqr_dot; phiLthetaL_ddot];
qdot = [xyz_dot; rpy_dot; phiLdot; thetaLdot; xyz_ddot; rpy_ddot; phiLthetaL_ddot];

end