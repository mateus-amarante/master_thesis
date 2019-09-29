startup

% PICK YOUR CONFIG FUNCTION
[physics_p, control_p, traj_p, sim_p, plot_p] = final_smc3d_slung_config();
% [physics_p, control_p, traj_p, sim_p, plot_p] = differentially_flat_config();
% [physics_p, control_p, traj_p, sim_p, plot_p] = smc3d_slung_config();

[t,x] = ode23(@(t, x) ode_fun(t, x, physics_p, control_p, traj_p, sim_p), sim_p.t, sim_p.x0);

% State remapping
q = x(:,1:end/2);
qdot = x(:,end/2+1:end);

% Compute desired trajectory
qd = traj_p.sample_fun(t);

u = zeros(length(t), control_p.n_inputs);

% TODO: define u for all timespecs at once
for i=1:length(t)
    u(i,:) = control_p.control_fun(x(i,:),qd(i,:),physics_p,control_p)';
end

plot_p.plot_state(t, q, qdot, qd, u, physics_p, control_p);
plot_p.plot_animation(t, q, qd, physics_p);
