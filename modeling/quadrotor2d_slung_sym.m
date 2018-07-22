
syms m M g L Itheta;
syms alpha theta;
syms x_dot z_dot theta_dot alpha_dot;
syms xc_dot zc_dot;
syms u1 u2;

q_dot = [x_dot; z_dot; theta_dot; alpha_dot];
qc_dot = [xc_dot; zc_dot; theta_dot; alpha_dot];

U = [u1; u2];

% DYNAMICS: Mq*q_ddot + Cq*q_dot + Gq = bq*u

% Inertia Matrix
Mq = [
    (M+m)            0               0      -m*L*cos(alpha);
    0                (M+m)           0       m*L*sin(alpha);
    0                0               Itheta  0;
    -m*L*cos(alpha)  m*L*sin(alpha)  0       m*L^2];

Mqc = [
    (M+m)            0               0       M*L*cos(alpha);
    0                (M+m)           0      -M*L*sin(alpha);
    0                0               Itheta  0;
    M*L*cos(alpha)  -M*L*sin(alpha)  0       M*L^2];

% Coriolis and Mentrifugal Forces
Cq = [0 0 0 m*L*sin(alpha)*alpha_dot; 0 0 0 m*L*cos(alpha)*alpha_dot; 0 0 0 0; 0 0 0 0];

Cqc = [0 0 0 -M*L*sin(alpha)*alpha_dot; 0 0 0 -M*L*cos(alpha)*alpha_dot; 0 0 0 0; 0 0 0 0];

% Gravity effect
Gq = [0; (M+m)*g; 0; m*g*L*sin(alpha)];

Gqc = [0; (M+m)*g; 0; -M*g*L*sin(alpha)];

% Input transformation
bq = [sin(theta) 0; cos(theta) 0; 0 1; 0 0];

bqc = [cos(theta-alpha)*sin(alpha) 0; cos(theta-alpha)*cos(alpha) 0; 0 1; m*L*sin(theta-alpha)/(M+m) 0];

% Output
q_ddot = collect(simplify(Mq\(-Cq*q_dot - Gq + bq*U)),U);

qc_ddot = collect(simplify(Mqc\(-Cqc*qc_dot - Gqc + bqc*U)),U);