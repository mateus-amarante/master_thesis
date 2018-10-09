function traj_p = shaped_poly_trajectory(t, q, n_vars, n_deriv_out, traj_p, physics_p)

temp_sample_fun = plan_polynomial_trajectory(t, q, n_vars, n_deriv_out);

M = physics_p.M;
m = physics_p.m;
g = physics_p.g;
L = physics_p.L;

wn = sqrt((M+m)*g/(M*L));
zeta = 0;

% traj_p.qd = zvd_shaper(traj_p.t,traj_p.qd,wn,zeta);

damping = sqrt(1-zeta^2);
wd = damping*wn;

% traj_p.qd = temp_sample_fun(traj_p.t);
% traj_p.sample_fun = temp_sample_fun;

Td = 2*pi/wd;
tt = (0:traj_p.dt:traj_p.Tf+Td)';
qd = temp_sample_fun(tt);
qd = zvd_shaper(tt,qd,wn,zeta);
traj_p.qd = qd(1:length(traj_p.t),:);

traj_p.sample_fun = @(t) interp1(traj_p.t,traj_p.qd,t,'linear',traj_p.qd(end));


end