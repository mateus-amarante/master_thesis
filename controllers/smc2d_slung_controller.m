function [u,s] = smc2d_slung_controller(q,qdot,xref,physics_p,control_p)
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
lambda_xdot = control_p.lambda_xdot;
lambda_thetadot = control_p.lambda_thetadot;

kappa_z = control_p.kappa_z;
eta_z = control_p.eta_z;

kappa_xtheta = control_p.kappa_xtheta;
eta_xtheta = control_p.eta_xtheta;

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

% ftheta = 0;
btheta = 1/Itheta;

% falpha = 0;
% balpha = -sin(alpha-theta)/(M*L);

% CONTROL CALCULATION
[U1, s1] = smc(z_d-z,zdot_d-zdot,zddot_d,fz,bz,lambda_z,kappa_z,eta_z);

if U1 > maxThrust
    U1 = maxThrust;
end

% From linearized model
% xddot = (-m/M*alpha + theta*(M+m)/M)*g;
% theta_c = (xddot_d + control_p.lambda_x*(x_d - x) + control_p.lambda_xdot*(xdot_d -xdot))*M/((M+m)*g);% + m*alpha/(M+m);
% thetadot_c = 0;
% thetadot_c = (control_p.lambda_x*(xdot_d - xdot) +control_p.lambda_xdot*(xddot_d - xddot))*M/((M+m)*g);% + m*alphadot/(M+m);

% theta_c = (xddot_d + control_p.lambda_x*(x_d - x) + control_p.lambda_xdot*(xdot_d -xdot))*M/((M+m*cos(alpha)^2)*g);
theta_c = 0;
thetadot_c = 0;
% den = g*( M + m*cos(alpha)^2 + (M - m*cos(alpha)*sin(alpha)*alphadot)*theta);
% thetadot_c = (control_p.lambda_x*(xdot_d - xdot) + control_p.lambda_xdot*(xddot_d - xddot))*M/den;

e = [x_d theta_c]' - [x theta]';
edot = [xdot_d thetadot_c]' - [xdot thetadot]';

f = [fx+bx*U1 0]';
b = [0 btheta]';
lambda = [lambda_x lambda_theta lambda_xdot lambda_thetadot]';
[U2, s2] = smcu([e;edot],[xddot_d 0]',f,b,lambda,kappa_xtheta,eta_xtheta);

u = [U1;U2];
s = [s1(:); s2(:)];
% Rotors velocities calculation
% u = sqrt([kt kt; -r*kt r*kt]\[U1; U2]);

end