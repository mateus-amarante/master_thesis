function [q_ddot] = quadrotor2d(q,qdot,u,physics_p)
% _2_      _1_
%  |________|
%
% |_ x(horizontal) z(vertical) y(in)
% theta (cw)

% q:     [x z theta]'
% q_dot: [x_dot z_dot theta_dot]'
% u:     [w1 w2]' (input)
% physics_p: physical parameters

% param remapping
M = physics_p.M;
g = physics_p.g;
Itheta = physics_p.Itheta;
% kt = physics_p.kt;
% r = physics_p.r;

% state remapping
theta = q(3);

% input remapping
% Fn = kt*(u'*u);
% My = r*kt*[-1 1]*(u.*u);
Fn = u(1);
My = u(2);

% dynamics
x_ddot = Fn*sin(theta)/M;
z_ddot = Fn*cos(theta)/M - g;
theta_ddot = My/Itheta;
d = zeros(size(q)); % known disturbance

% output remapping
q_ddot = [x_ddot z_ddot theta_ddot]' + d;

end