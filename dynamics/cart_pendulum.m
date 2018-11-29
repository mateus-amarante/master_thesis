function [q_ddot] = quadrotor2d_slung(q,q_dot,u,params)
%      __
%     |__|  -->
%      /
%     /
%    /
%   O
%

% Param remapping
g = params.g;
m = params.m;
M = params.M;
L = params.L;

% State remapping
% x = q(1);
% z = q(2);
theta = q(3);
alpha = q(4);

% x_dot = q_dot(1);
% z_dot = q_dot(2);
% theta_dot = q_dot(3);
alpha_dot = q(4);

% Input remapping
U(1,1) = kt*(u'*u);
U(2,1) = r*kt*[-1 1]*(u.*u);

% DYNAMICS

% Inertia Matrix
Mq = [
    (M+m)            0               0      -m*L*cos(alpha);
    0                (M+m)           0       m*L*sin(alpha);
    0                0               Itheta  0;
    -m*L*cos(alpha)  m*L*sin(alpha)  0       m*L^2];

% Coriolis and Mentrifugal Forces
Cq = zeros(4,4);
Cq(1,4) = m*L*sin(alpha)*alpha_dot;
Cq(2,4) = m*L*cos(alpha)*alpha_dot;

% Gravity effect
Gq = [0; (M+m)*g; 0; m*g*L*sin(alpha)];

% Input transformation
bq = zeros(4,2);
bq(1,1) = sin(theta);
bq(2,1) = cos(theta);
bq(3,2) = 1;
d = zeros(4,1); % known disturbance

% Equation solving (Mq*q_ddot + Cq*q_dot + Gq = bq*u)
q_ddot = Mq\(-Cq*q_dot - Gq + bq*U) + d;

end