function [u,s] = full_smc2d_slung_controller(q,qdot,xref,physics_p,control_p)
% SMC for 2D quadrotor
% q: [x z theta]'

% Params renaming
M = physics_p.M;
g = physics_p.g;
Itheta = physics_p.Itheta;
maxThrust = physics_p.maxThrust;

L = physics_p.L;
m = physics_p.m;

% Eigen values of alpha, alphadot and z
gamma_alpha = control_p.gamma_alpha;
gamma_alphadot = control_p.gamma_alphadot;
gamma_z = control_p.gamma_z;

kappa_zalpha = control_p.kappa_zalpha;
eta_zalpha = control_p.eta_zalpha;

% USMC x theta
lambda_x = control_p.lambda_x;
lambda_theta = control_p.lambda_theta;
lambda_xdot = control_p.lambda_xdot;
lambda_thetadot = control_p.lambda_thetadot;

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

% Sliding variable constants

% auxiliar variables
A = M*L*cos(theta);
B = (M + m)*sin(theta);
% C = L*(M + m)*(g + zddot_d)*(M-m*sin(theta)^2);

% "Constants"
lambda_alphadot = control_p.lambda_alphadot;
lambda_alpha = control_p.lambda_alpha;
lambda_zdot = (gamma_alpha*lambda_alphadot - lambda_alpha*B)/(gamma_z/gamma_alphadot - gamma_alpha);
lambda_z = gamma_z/gamma_alphadot*lambda_zdot;


% Dynamic model functions
fx = -m*L*sin(alpha)*alphadot^2/(M+m);
bx = (-m*cos(alpha)*sin(alpha-theta) + M*sin(theta))/(M*(M+m));

fz = -m*L*cos(alpha)*alphadot^2/(M+m) - g;
bz = (m*sin(alpha)*sin(alpha-theta) + M*cos(theta))/(M*(M+m));

ftheta = 0;
btheta = 1/Itheta;

falpha = 0;
balpha = -sin(alpha-theta)/(M*L);

% SMCU z and alpha
lambda_zalpha = [lambda_z; lambda_alpha]; 
lambda_zalphadot = [lambda_zdot; lambda_alphadot];

e_zalpha = [z_d - z; 0 - alpha];
edot_zalpha = [zdot_d - zdot; 0 - alphadot];

zalphaddot_d = [zddot_d; 0];

fzalpha = [fz; falpha];
bzalpha = [bz; balpha];

[U1, s1] = smcu([e_zalpha;edot_zalpha],zalphaddot_d,fzalpha,bzalpha,...
    [lambda_zalpha; lambda_zalphadot],kappa_zalpha,eta_zalpha);

if U1 > maxThrust
    U1 = maxThrust;
end

% From linearized model
theta_c = 0;
thetadot_c = 0;

e = [x_d theta_c]' - [x theta]';
edot = [xdot_d thetadot_c]' - [xdot thetadot]';

f = [fx+bx*U1 0]';
b = [ftheta btheta]';
lambda = [lambda_x lambda_theta lambda_xdot lambda_thetadot]';
[U2, s2] = smcu([e;edot],[xddot_d 0]',f,b,lambda,kappa_xtheta,eta_xtheta);

u = [U1;U2];
s = [s1(:); s2(:)];
% Rotors velocities calculation
% u = sqrt([kt kt; -r*kt r*kt]\[U1; U2]);

end

