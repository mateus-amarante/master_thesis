function u = smc2d_slung_controller(q,qdot,xref,physics_p,control_p)
% SMC for 2D quadrotor
% q: [x z theta]'

%% Input Remmaping

% Physical parameters renaming
M = physics_p.M;
g = physics_p.g;
I = physics_p.I;
Ix = physics_p.Ix;
Iy = physics_p.Iy;
Iz = physics_p.Iz;
maxThrust = physics_p.maxThrust;

L = physics_p.L;
m = physics_p.m;

% Control parameters renaming
lambda_zpsi = contro_p.lambda_zpsi;
kappa_zpsi = contro_p.kappa_zpsi;
eta_zpsi = contro_p.eta_zpsi;

lambda_xtheta = control_p.lambda_xtheta;
lambda_xtheta_dot = control_p.lambda_xtheta_dot;
kappa_xtheta = contro_p.kappa_xtheta;
eta_xtheta = contro_p.eta_xtheta;

lambda_yphi = control_p.lambda_yphi;
lambda_yphi_dot = control_p.lambda_yphi_dot;
kappa_yphi = contro_p.kappa_yphi;
eta_yphi = contro_p.eta_yphi;

% Reference signal renaming
q_d = xref(1:4);
qdot_d = xref(5:8);
qddot_d = xref(9:12);

x_d, xdot_d, xddot_d = q_d(1), qdot_d(1), qddot_d(1);
y_d, ydot_d, yddot_d = q_d(2), qdot_d(2), qddot_d(2);
z_d, zdot_d, zddot_d = q_d(3), qdot_d(3), qddot_d(3);
psi_d, psidot_d, psiddot_d = q_d(4), qdot_d(4), qddot_d(4);

% State renaming
x, xdot = q(1), qdot(1);
y, ydot = q(2), qdot(2);
z, zdot = q(3), qdot(3);

phi, phidot = q(4), qdot(4);
theta, thetadot = q(5), qdot(5);
psi, psidot = q(6), qdot(6);

phiL, phiLdot = q(7), qdot(7);
thetaL, thetaLdot = q(8), qdot(8);

%% Dynamic model state-space representation

% Helper expressions
sphiL = sin(phiL); sthetaL = sin(thetaL);
cphiL = cos(phiL); cthetaL = cos(thetaL);

ux = cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
uy = cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
uz = cos(phi)*cos(theta);

Cf = m*L/(M+m)*(cthetaL^2*phiLdot^2 + thetaLdot^2);
Cb = m/(M*(M + m));

% Functions
fx = -Cf*sthetaL;
bx = Cb*(sthetaL*cthetaL*(uy*sphiL - uz*cphiL) + ux*(M/m + cthetaL^2));

fy = Cf*sphiL*cthetaL;
by = Cb*(sphiL*cthetaL*(ux*sthetaL + uz*cphiL*cthetaL) + uy*(M/m + 1 - sphiL^2*cthetaL^2));

fz = -Cf*cphiL*cthetaL - g;
bz = Cb*(cphiL*cthetaL*(-ux*sthetaL + uy*sphiL*cthetaL) + uz*(M/m + 1 - cphiL^2*cthetaL^2));

fphi = thetadot*psidot(Iy - Iz)/Ix;
bphi = 1/Ix;

ftheta = phidot*psidot(Iz - Ix)/Iy;
btheta = 1/Iy;

fpsi = phidot*thetadot(Ix - Iy)/Iz;
bpsi = 1/Iz;

fphiL = 2*tan(thetaL)*phiLdot*thetaLdot;
bphiL = -(uy*cphiL + uz*sphiL)/(M*L*cthetaL);

fthetaL = -sthetaL*cthetaL*phiLdot^2;
bthetaL = (ux*cthetaL + sthetaL*(uy*sphiL - uz*cphiL))/(M*L);

%% Control input calculation

u = zeros(4, 1);

% Fully-actuated SMC: z psi control
ezpsi = [z_d; psi_d] - [z; psi];
ezpsi_dot = [zdot_d; psidot_d] - [zdot; psidot];

zpsi_ddot_d = [zddot_d; psiddot_d];

fzpsi = [fz; fpsi];
bzpsi = [bz; bpsi];

u([1 4]) = smc([ezpsi, ezpsi_dot, zpsi_ddot_d, fzpsi, bzpsi, lambda_zpsi, kappa_zpsi, eta_zpsi);

if u(1) > maxThrust
    u(1) = maxThrust;
end

% Under-actuated SMC: x and theta
theta_d, thetadot_d, thetaddot_d = 0,0,0;

extheta = [x_d; theta_d] - [x; theta];
extheta_dot = [xdot_d; thetadot_d] - [xdot; thetadot];

xtheta_ddot_d = [xddot_d; thetaddot_d];

fxtheta = [fx + bx*u(1); ftheta];
bxtheta = [0; btheta];

u(3) = smcu([extheta; extheta_dot], xtheta_ddot_d, fxtheta, bxtheta, [lambda_xtheta; lambda_xtheta_dot] , kappa_xtheta, eta_xtheta);

% Under-actuated SMC: y and phi
phi_d, phidot_d, phiddot_d = 0,0,0;

eyphi = [y_d; phi_d] - [y; phi];
eyphi_dot = [ydot_d; phidot_d] - [ydot; phidot];

yphi_ddot_d = [yddot_d; phiddot_d];

fyphi = [fy + by*u(1); fphi];
byphi = [0; bphi];

u(2) = smcu([eyphi; eyphi_dot], yphi_ddot_d, fyphi, byphi, [lambda_yphi; lambda_yphi_dot] , kappa_yphi, eta_yphi);

end