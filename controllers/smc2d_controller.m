function u = smc2d_controller(q,qdot,xref,physics_p,control_p)
% SMC for 2D quadrotor
% q: [x z theta]'

% Params renaming
M = physics_p.M;
g = physics_p.g;
Itheta = physics_p.Itheta;
% kt = physics_p.kt;
% r = physics_p.r;
maxThrust = physics_p.maxThrust;

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


fz = -g;
bz = cos(theta)/M;

% Control calculation
U1 = smc(z_d-z,zdot_d-zdot,zddot_d,fz,bz,lambda_z,kappa_z,eta_z);

if U1 > maxThrust
    U1 = maxThrust;
end

fx = sin(theta)/M*U1;
ftheta = 0;

bx = 0;
btheta = 1/Itheta;

theta_d = asin(xddot_d*M/U1);

e = [x_d theta_d]' - [x theta]';
edot = [xdot_d 0]' - [xdot thetadot]';

f = [fx ftheta]';
b = [bx btheta]';

lambda = [lambda_x lambda_theta lambda_xdot lambda_thetadot]';

U2 = smcu([e;edot],[xddot_d 0]',f,b,lambda,kappa_xtheta,eta_xtheta);

u = [U1; U2];

% Rotors velocities calculation
% u = sqrt([kt kt; -r*kt r*kt]\[U1; U2]);

end