function u = nested2d_controller(q,qdot,xref,physics_p,control_p)
% https://www.youtube.com/watch?v=uPsenrdsu28
% q: [x z theta]'

% Params renaming
M = physics_p.M;
g = physics_p.g;
Itheta = physics_p.Itheta;
kt = physics_p.kt;
r = physics_p.r;
maxThrust = physics_p.maxThrust;

kp_x = control_p.kp_x;
kd_x = control_p.kd_x;
kp_z = control_p.kp_z;
kd_z = control_p.kd_z;
kp_theta = control_p.kp_theta;
kd_theta = control_p.kd_theta;

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

% Control calculation
U1 = M*(g + zddot_d +kp_z*(z_d - z) +kd_z*(zdot_d - zdot));

if U1 > maxThrust
    U1 = maxThrust;
end
    
xddot = g*theta;
theta_c = (xddot_d + kp_x*(x_d - x) + kd_x*(xdot_d -xdot))/g;
thetadot_c = (kp_x*(xdot_d - xdot) +kd_x*(xddot_d - xddot))/g;

U2 = Itheta*(kp_theta*(theta_c - theta) + kd_theta*(thetadot_c - thetadot));

u = [U1; U2];

% Rotors velocities calculation
% u = sqrt([kt kt; -r*kt r*kt]\[U1; U2]);

end