startup

% PICK YOUR CONFIG FUNCTION
% [physics_p, control_p, traj_p, sim_p, plot_p] = nested2d_config();
% [physics_p, control_p, traj_p, sim_p, plot_p] = smc2d_config();
% [physics_p, control_p, traj_p, sim_p, plot_p] = nested2d_slung_config();
% [physics_p, control_p, traj_p, sim_p, plot_p] = smc2d_slung_config();
% [physics_p, control_p, traj_p, sim_p, plot_p] = smc2d_slung_config2();
% [physics_p, control_p, traj_p, sim_p, plot_p] = smc3d_slung_config();
% [physics_p, control_p, traj_p, sim_p, plot_p] = nested3d_config();
% [physics_p, control_p, traj_p, sim_p, plot_p] = nested3d_slung_config();
[physics_p, control_p, traj_p, sim_p, plot_p] = smc3d_slung_flat_config();

[t,x] = ode45(@(t, x) ode_fun(t, x, physics_p, control_p, traj_p, sim_p), sim_p.t, sim_p.x0);

% State remapping
q = x(:,1:end/2);
qdot = x(:,end/2+1:end);

% Compute desired trajectory
qd = traj_p.sample_fun(t);

% TODO: define u for all timespecs at once
for i=1:length(t)
    % FIXME: state variables are temporarily transposed for nested3d_control
    u(i,:) = control_p.control_fun(q(i,:)',qdot(i,:)',qd(i,:)',physics_p,control_p)';
end

plot_p.plot_state(t, q, qdot, qd, u, physics_p, control_p);
plot_p.plot_animation(t, q, qd, physics_p);
