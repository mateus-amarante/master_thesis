function [u,s] = smc2d_slung_controller2(q,qdot,xref,physics_p,control_p)
% SMC for 2D quadrotor
% q: [x z theta]'

% Params renaming
M = physics_p.M;
g = physics_p.g;
Itheta = physics_p.Itheta;
kt = physics_p.kt;
r = physics_p.r;
maxThrust = physics_p.maxThrust;

L = physics_p.L;
m = physics_p.m;

lambda_z = control_p.lambda_z;
lambda_x = control_p.lambda_x;
lambda_theta = control_p.lambda_theta;


kappa_z = control_p.kappa_z;
kappa_x = control_p.kappa_x;
kappa_theta = control_p.kappa_theta;

eta_z = control_p.eta_z;
eta_x = control_p.eta_x;
eta_theta = control_p.eta_theta;

% Reference renaming
qd = xref(1:2);
qdot_d = xref(3:4);
qddot_d = xref(5:6);

x_d = qd(1);
xdot_d = qdot_d(1);
xddot_d = qddot_d(1);

z_d = qd(2);
zdot_d = qdot_d(2);
zddot_d = qddot_d(2);

% State renaming
x = q(1);
xdot = qdot(1);
z = q(2);
zdot = qdot(2);
theta = q(3);
thetadot =  qdot(3);

alpha = q(4);
alphadot = qdot(4);

% Dynamic model functions
fx = -m*L*sin(alpha)*alphadot^2/(M+m);
bx = (-m*cos(alpha)*sin(alpha-theta) + M*sin(theta))/(M*(M+m));

fz = -m*L*cos(alpha)*alphadot^2/(M+m) - g;
bz = (m*sin(alpha)*sin(alpha-theta) + M*cos(theta))/(M*(M+m));

ftheta = 0;
btheta = 1/Itheta;

% falpha = 0;
% balpha = -sin(alpha-theta)/(M*L);

% CONTROL CALCULATION
[U1, s1] = smc(z_d-z,zdot_d-zdot,zddot_d,fz,bz,lambda_z,kappa_z,eta_z);

% if U1 > maxThrust
%     U1 = maxThrust;
% end

fxx = fx + (bx - M*sin(theta)/(M*(M+m)))*U1;
bxx = 1/(M+m);

[U1x, s2] = smc(x_d-x,xdot_d-xdot,xddot_d,fxx,bxx,lambda_x,kappa_x,eta_x);

theta_c = atan(U1x/U1);
thetadot_c = 0;

[U2, s3] = smc(theta_c-theta,-thetadot,0,ftheta,btheta,lambda_theta,kappa_theta,eta_theta);

u = [U1;U2];
s = [s1(:); s2(:);s3(:)];
% Rotors velocities calculation
% u = sqrt([kt kt; -r*kt r*kt]\[U1; U2]);

end