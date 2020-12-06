function xdot = ode_fun(t, x, physics_p, control_p, traj_p, sim_p)

x_sensor = x + sim_p.noise(t);

% trajectory sampling
xref = traj_p.sample_fun(t);

% control
u = control_p.control_fun(x_sensor, xref, physics_p, control_p);

% dynamics
xdot = sim_p.dyn_fun(x, u, physics_p);

end