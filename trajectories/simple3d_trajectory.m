function traj_p = simple3d_trajectory(x,y,z,psi,T,steady_time,dt)

% Trajectory parameters
traj_p.pos = [0 0 0 0; x y z psi];
traj_p.vel = zeros(size(traj_p.pos));
traj_p.acc = traj_p.vel;
traj_p.n_vars = size(traj_p.pos,2);
traj_p.qd = [traj_p.pos traj_p.vel traj_p.acc];

traj_p.Tf = T + steady_time;
traj_p.traj_tspan = [0 T]';
% traj_p.traj_t = traj_p.tspan(1):traj_p.dt:traj_p.tspan(2);

% Simulation parameters
traj_p.tspan = [0 traj_p.Tf]';
traj_p.dt = dt;
traj_p.t = (0:dt:traj_p.Tf)';
traj_p.x0 = zeros(12,1);

% Trajectory building
traj_p.sample_fun = plan_polynomial_trajectory(traj_p.traj_tspan, traj_p.qd, traj_p.n_vars, 3);
traj_p.qd = traj_p.sample_fun(traj_p.t);

end

