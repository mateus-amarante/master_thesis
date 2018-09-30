function [qq_d, q_d, qdot_d, qddot_d] = sample_polysegment(coef,t)
% coef = [a0 a1 a2 a3 a4 a5]' per var (column) for 'n' variables
% a0 + a1*t + a2*t² + a3*t³ + a4*t? + a5*t?
% qd =[pos1 pos2 ... posn,
%      vel1 vel2 ... veln,
%      acc1 acc2 ... accn] per time (line) for 'n' variables

nvar    = size(coef, 2);
n_coefs = size(coef, 1);

t1 = t(:)';
t0 = ones(size(t1));
zero = zeros(size(t1));

qdot_d  = [];
qddot_d = [];

if n_coefs >= 4
    t2 = t1.^2;
    t3 = t1.^3;
    
    if n_coefs == 6 % QUINTIC
        t4 = t1.^4;
        t5 = t1.^5;
        
        t_pos = [t0;      t1;   t2;   t3;    t4;    t5];
        t_vel = [ zero;   t0; 2*t1; 3*t2;  4*t3;  5*t4];
        t_acc = [ zero; zero; 2*t0; 6*t1; 12*t2; 20*t3];

        qddot_d = (coef'*t_acc)'; % acceleration
        
    else % CUBIC
        t_pos = [t0;      t1;   t2;   t3];
        t_vel = [ zero;   t0; 2*t1; 3*t2];
    end
    
    qdot_d  = (coef'*t_vel)'; % velocity
else % LINEAR
    t_pos = [t0; t1];
end

q_d = (coef'*t_pos)'; % position

qq_d = [q_d qdot_d qddot_d]; % position, velocity and acceleration
end
