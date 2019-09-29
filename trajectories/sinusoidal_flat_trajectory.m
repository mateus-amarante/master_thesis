function traj_p = simple_smooth_trajectory(qd, T, wait_time, steady_time)

qd = qd(:)';

% Trajectory parameters
traj_p.pos = [zeros(size(qd)); qd];
traj_p.vel = zeros(size(traj_p.pos));
traj_p.acc = traj_p.vel;

traj_p.n_vars = size(traj_p.pos,2);
traj_p.qd = [traj_p.pos traj_p.vel traj_p.acc];

traj_p.Tf = wait_time + T + steady_time;
traj_p.traj_tspan = [wait_time, T + wait_time]';

% Trajectory building
traj_p.sample_fun = plan_polynomial_trajectory2(traj_p.traj_tspan, traj_p.qd, traj_p.n_vars, 2);

end

