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
Ix = physics_p.Ix;
Iy = physics_p.Iy;
Iz = physics_p.Iz;
I  = physics_p.I;
Iinv = physics_p.Iinv;
kt = physics_p.kt;
km = physics_p.km;
r = physics_p.r;

% state remapping
xyz     = q(1:3);
rpy     = q(4:6);
beta_alpha = q(7:8);
xyz_dot = qdot(1:3);
rpy_dot = qdot(4:6);
beta_alpha_dot = qdot(7:8);

alpha = beta_alpha(1);
beta = beta_alpha(2);
alphadot = beta_alpha_dot(1);
betadot = beta_alpha_dot(2);

phi = rpy(1);
theta = rpy(2);
psi = rpy(3);

phidot = rpy_dot(1);
thetadot = rpy_dot(2);
psidot = rpy_dot(3);


% input remapping
U = zeros(4,1);
U(1) = kt*(u'*u);
wr2 = (u.*u);
U(2) = kt*r*[0 1 0 -1]*wr2;
U(3) = kt*r*[-1 0 1 0]*wr2;
U(4) = km*[ 1 -1 1 -1]*wr2;

% Linear Dynamics
salpha = sin(alpha);
calpha = cos(alpha);
sbeta = sin(beta);
cbeta = cos(beta);

m14 =  m*L*calpha*cbeta; m41 = m14;
m15 = -m*L*salpha*sbeta; m51 = m15;
m24 =  m*L*calpha*sbeta; m42 = m24;
m25 =  m*L*salpha*cbeta; m52 = m25;
m34 =  m*L*salpha; m43 = m34;

Mq = [
    M+m  0    0    m14    m15;
    0    M+m  0    m24    m25;
    0    0    M+m  m34    0;
    m41  m42  m43  m*L^2  0;
    m51  m52  0    0,     m*L^2*salpha^2];

c14 = -m*L*(calpha*sbeta*betadot  + salpha*cbeta*alphadot);
c15 = -m*L*(calpha*sbeta*alphadot + salpha*cbeta*betadot);
c24 =  m*L*(calpha*cbeta*betadot  - salpha*sbeta*alphadot);
c25 =  m*L*(calpha*cbeta*alphadot - salpha*sbeta*betadot);
c34 =  m*L*calpha*alphadot;
c45 = -m*L^2*salpha*calpha*betadot;
c54 = -c45;
c55 = m*L^2*salpha*calpha*alphadot;

Cq = [
    0 0 0 c14 c15;
    0 0 0 c24 c25;
    0 0 0 c34 0;
    0 0 0 0   c45;
    0 0 0 c54 c55];

Gq = [0; 0; (M+m)*g; m*g*L*salpha; 0];

ux = cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
uy = cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);

bq = [ux uy cos(phi)*cos(theta) 0 0]';


linear_acc = Mq\(-Cq*[xyz_dot;beta_alpha_dot] -Gq + bq*U(1));


% Angular Dynamics
Tib = Ti2b(phi,theta);
Tbi_dot = Tb2i_dot(phi,theta,phidot,thetadot);
pqr = Tib*rpy_dot;

pqr_dot = Iinv*(U(2:4) + cross(pqr,I*pqr));
rpy_ddot = Tib*(pqr_dot-Tbi_dot*rpy_dot);

% Output remapping
qddot = [linear_acc(1:3); rpy_ddot; linear_acc(4:5)];

end