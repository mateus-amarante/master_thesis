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

% Reference signal renaming
q_d = xref(1:4);
qdot_d = xref(5:8);
qddot_d = xref(9:12);

x_d = q_d(1); xdot_d = qdot_d(1); xddot_d = qddot_d(1);
y_d = q_d(2); ydot_d = qdot_d(2); yddot_d = qddot_d(2);
z_d = q_d(3); zdot_d = qdot_d(3); zddot_d = qddot_d(3);
psi_d = q_d(4); psidot_d = qdot_d(4); psiddot_d = qddot_d(4);

% State renaming
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

Cf = m*L/(M+m)*(cthetaL^2*phiLdot^2 + thetaLdot^2);
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

u([1 4]) = smc(ezpsi, ezpsi_dot, zpsi_ddot_d, fzpsi, bzpsi, lambda_zpsi, kappa_zpsi, eta_zpsi);

if u(1) > maxThrust
    u(1) = maxThrust;
end

% Compute desired phi and theta
theta_d = 0; thetadot_d = 0; thetaddot_d = 0;
phi_d = 0; phidot_d = 0; phiddot_d = 0;

% xddot_c = xddot_d + lambda_xtheta(1)*(x_d - x) + lambda_xtheta_dot(1)*(xdot_d - xdot);
% yddot_c = yddot_d + lambda_yphi(1)*(y_d - y) + lambda_yphi_dot(1)*(ydot_d - ydot);
% phi_d = -M/((M + m)*g)*(sin(psi)*xddot_c - cos(psi)*yddot_c);
% theta_d = M/((M + m)*g)*(sin(psi)*xddot_c + cos(psi)*yddot_c);

% Compute controller variables
psiddot = fpsi + bpsi*u(4);
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

exbdot = psidot*eyb + cos(psi)*exdot + sin(psi)*eydot;
eybdot = -psidot*exb - sin(psi)*exdot + cos(psi)*eydot;

exbddot = psiddot*eyb - psidot^2*exb + 2*psidot*(-sin(psi)*exdot + cos(psi)*eydot) + ...
    cos(psi)*exddot + sin(psi)*eyddot;

eybddot = -psiddot*exb - psidot^2*eyb - 2*psidot*(cos(psi)*exdot + sin(psi)*eydot) + ...
    -sin(psi)*exddot + cos(psi)*eyddot;

% Under-actuated SMC: xb and theta
exbtheta = [exb; theta_d - theta];
exbtheta_dot = [exbdot; thetadot_d - thetadot];

xbtheta_ddot_d = [exbddot; thetaddot_d];

fxbtheta = [0; ftheta];
bxbtheta = [0; btheta];

u(3) = smcu([exbtheta; exbtheta_dot], xbtheta_ddot_d, fxbtheta, bxbtheta, [lambda_xtheta; lambda_xtheta_dot] , kappa_xtheta, eta_xtheta);

% Under-acrtuated SMC: yb and phi
eybphi = [eyb; phi_d - phi];
eybphi_dot = [eybdot; phidot_d - phidot];

ybphi_ddot_d = [eybddot; phiddot_d];

fybphi = [0; fphi];
bybphi = [0; bphi];

u(2) = smcu([eybphi; eybphi_dot], ybphi_ddot_d, fybphi, bybphi, [lambda_yphi; lambda_yphi_dot] , kappa_yphi, eta_yphi);

% % Under-actuated SMC: x and theta
% extheta = [x_d; theta_d] - [x; theta];
% extheta_dot = [xdot_d; thetadot_d] - [xdot; thetadot];
% 
% xtheta_ddot_d = [xddot_d; thetaddot_d];
% 
% fxtheta = [fx + bx*u(1); ftheta];
% bxtheta = [0; btheta];
% 
% u(3) = smcu([extheta; extheta_dot], xtheta_ddot_d, fxtheta, bxtheta, [lambda_xtheta; lambda_xtheta_dot] , kappa_xtheta, eta_xtheta);
% 
% % Under-actuated SMC: y and phi
% eyphi = [y_d; phi_d] - [y; phi];
% eyphi_dot = [ydot_d; phidot_d] - [ydot; phidot];
% 
% yphi_ddot_d = [yddot_d; phiddot_d];
% 
% fyphi = [fy + by*u(1); fphi];
% byphi = [0; bphi];
% 
% u(2) = smcu([eyphi; eyphi_dot], yphi_ddot_d, fyphi, byphi, [lambda_yphi; lambda_yphi_dot] , kappa_yphi, eta_yphi);

end