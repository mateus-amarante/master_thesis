function traj_p = setpoint_trajectory(setpoint, Tf, dt)

traj_p.t = 0:dt:Tf;

setpoint = setpoint(:);

traj_p.sample_fun = @(t) setpoint'.*ones(length(t), length(setpoint));
traj_p.qd = traj_p.sample_fun(traj_p.t);

end

