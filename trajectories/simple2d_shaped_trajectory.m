function traj_p = simple2d_shaped_trajectory(x,z,T,steady_time,dt,physics_p)

traj_p = simple2d_trajectory(x,z,T,steady_time,dt);

M = physics_p.M;
m = physics_p.m;
g = physics_p.g;
L = physics_p.L;

wn = sqrt((M+m)*g/(M*L));
zeta = 0;

% traj_p.qd = zvd_shaper(traj_p.t,traj_p.qd,wn,zeta);

damping = sqrt(1-zeta^2);
wd = damping*wn;
Td = 2*pi/wd;
tt = (0:dt:traj_p.Tf+Td)';

qd = traj_p.sample_fun(tt)';
qd = zvd_shaper(tt,qd,wn,zeta);
traj_p.qd = qd(1:length(traj_p.t),:);

traj_p.sample_fun = @(t)interp1(traj_p.t,traj_p.qd,t,'linear',traj_p.qd(end));

end

