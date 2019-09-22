function [u, s, ueq, usw] = final_smc3d_slung_controller(q,xref,physics_p,control_p)
% SMC for quadrotor with suspended load systemfg
% q: [x z theta]'

%% Physical parameters renaming
M = physics_p.M;
g = physics_p.g;
% I = physics_p.I;
Ix = physics_p.Ix;
Iy = physics_p.Iy;
Iz = physics_p.Iz;
% maxThrust = physics_p.maxThrust;

l = physics_p.l;
m = physics_p.m;

%% Control parameters renaming
lambda_zpsi = control_p.lambda_zpsi;
kappa_zpsi = control_p.kappa_zpsi;
eta_zpsi = control_p.eta_zpsi;

lambda_xtheta = control_p.lambda_xtheta;
lambda_xtheta_dot = control_p.lambda_xtheta_dot;
kappa_xtheta = control_p.kappa_xtheta;
eta_xtheta = control_p.eta_xtheta;

lambda_yphi = control_p.lambda_yphi;
lambda_yphi_dot = control_p.lambda_yphi_dot;
kappa_yphi = control_p.kappa_yphi;
eta_yphi = control_p.eta_yphi;

%% Reference signal renaming
q_d = xref(1:8);
qdot_d = xref(9:16);
qddot_d = xref(17:24);

x_d = q_d(1); xdot_d = qdot_d(1); xddot_d = qddot_d(1);
y_d = q_d(2); ydot_d = qdot_d(2); yddot_d = qddot_d(2);
z_d = q_d(3); zdot_d = qdot_d(3); zddot_d = qddot_d(3);

phi_d = q_d(4); phidot_d = q_d(4); phiddot_d = q_d(4);
theta_d = q_d(5); thetadot_d = q_d(5); thetaddot_d = q_d(5);
psi_d = q_d(6); psidot_d = qdot_d(6); psiddot_d = qddot_d(6);

% phiL_d = q_d(7); phiLdot_d = q_d(7); phiLddot_d = q_d(7);
% thetaL_d = q_d(8); thetaLdot_d = q_d(8); thetaLddot_d = q_d(8);

%% State renaming
qdot = q(9:16);
x = q(1); xdot = qdot(1);
y = q(2); ydot = qdot(2);
z = q(3); zdot = qdot(3);

phi = q(4); phidot = qdot(4);
theta = q(5); thetadot = qdot(5);
psi = q(6); psidot = qdot(6);

phiL = q(7); phiLdot = qdot(7);
thetaL = q(8); thetaLdot = qdot(8);

%% Dynamic model state-space representation

% Helper expressions
sphiL = sin(phiL); sthetaL = sin(thetaL);
cphiL = cos(phiL); cthetaL = cos(thetaL);

ux = cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
uy = cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
uz = cos(phi)*cos(theta);

Cf = m*l/(M+m)*(cthetaL^2*phiLdot^2 + thetaLdot^2);
Cb = m/(M*(M + m));

% Functions
fx = -Cf*sthetaL;
bx = Cb*(sthetaL*cthetaL*(uy*sphiL - uz*cphiL) + ux*(M/m + cthetaL^2));

fy = Cf*sphiL*cthetaL;
by = Cb*(sphiL*cthetaL*(ux*sthetaL + uz*cphiL*cthetaL) + uy*(M/m + 1 - sphiL^2*cthetaL^2));

fz = -Cf*cphiL*cthetaL - g;
bz = Cb*(cphiL*cthetaL*(-ux*sthetaL + uy*sphiL*cthetaL) + uz*(M/m + 1 - cphiL^2*cthetaL^2));

fphi = thetadot*psidot*(Iy - Iz)/Ix;
bphi = 1/Ix;

ftheta = phidot*psidot*(Iz - Ix)/Iy;
btheta = 1/Iy;

fpsi = phidot*thetadot*(Ix - Iy)/Iz;
bpsi = 1/Iz;

% fphiL = 2*tan(thetaL)*phiLdot*thetaLdot;
% bphiL = -(uy*cphiL + uz*sphiL)/(M*l*cthetaL);

% fthetaL = -sthetaL*cthetaL*phiLdot^2;
% bthetaL = (ux*cthetaL + sthetaL*(uy*sphiL - uz*cphiL))/(M*l);

%% Control input calculation

u = zeros(4, 1);

% Fully-actuated SMC: z psi control
ezpsi = [z_d; psi_d] - [z; psi];
ezpsi_dot = [zdot_d; psidot_d] - [zdot; psidot];

zpsi_ddot_d = [zddot_d; psiddot_d];

fzpsi = [fz; fpsi];
bzpsi = [bz; bpsi];

[u([1 4]), s_zpsi, ueq_zpsi, usw_zpsi]  = smc(ezpsi, ezpsi_dot, zpsi_ddot_d, fzpsi, bzpsi, lambda_zpsi, kappa_zpsi, eta_zpsi);

% if u(1) > maxThrust
%     u(1) = maxThrust;
% end

% Compute controller variables
% psiddot = fpsi + bpsi*u(4);
xddot = fx + bx*u(1);
yddot = fy + by*u(1);

ex = x_d - x;
exdot = xdot_d - xdot;
exddot = xddot_d - xddot;

ey = y_d - y;
eydot = ydot_d - ydot;
eyddot = yddot_d - yddot;

exb =  cos(psi)*ex + sin(psi)*ey;
eyb = -sin(psi)*ex + cos(psi)*ey;

exbdot =  cos(psi)*exdot + sin(psi)*eydot;
eybdot = -sin(psi)*exdot + cos(psi)*eydot;

exbddot =  cos(psi)*exddot + sin(psi)*eyddot;
eybddot = -sin(psi)*exddot + cos(psi)*eyddot;

% exbdot = psidot*eyb + cos(psi)*exdot + sin(psi)*eydot;
% eybdot = -psidot*exb - sin(psi)*exdot + cos(psi)*eydot;

% exbddot = psiddot*eyb - psidot^2*exb + 2*psidot*(-sin(psi)*exdot + cos(psi)*eydot) + ...
%     cos(psi)*exddot + sin(psi)*eyddot;

% eybddot = -psiddot*exb - psidot^2*eyb - 2*psidot*(cos(psi)*exdot + sin(psi)*eydot) + ...
%     -sin(psi)*exddot + cos(psi)*eyddot;

% Under-actuated SMC: xb and theta
exbtheta = [exb; theta_d - theta];
exbtheta_dot = [exbdot; thetadot_d - thetadot];

xbtheta_ddot_d = [exbddot; thetaddot_d];

fxbtheta = [0; ftheta];
bxbtheta = [0; btheta];

[u(3), s_xtheta, ueq_xtheta, usw_xtheta] = smcu([exbtheta; exbtheta_dot], xbtheta_ddot_d, fxbtheta, bxbtheta, [lambda_xtheta; lambda_xtheta_dot] , kappa_xtheta, eta_xtheta);

% Under-acrtuated SMC: yb and phi
eybphi = [eyb; phi_d - phi];
eybphi_dot = [eybdot; phidot_d - phidot];

ybphi_ddot_d = [eybddot; phiddot_d];

fybphi = [0; fphi];
bybphi = [0; bphi];

[u(2), s_yphi, ueq_yphi, usw_yphi] = smcu([eybphi; eybphi_dot], ybphi_ddot_d, fybphi, bybphi, [lambda_yphi; lambda_yphi_dot] , kappa_yphi, eta_yphi);

s = [s_zpsi(:); s_xtheta(:); s_yphi(:)];
ueq = [ueq_zpsi(:); ueq_xtheta(:); ueq_yphi(:)];
usw = [usw_zpsi(:); usw_xtheta(:); usw_yphi(:)];

end