function x_dot = ode_fun(t, x, physics_p, control_p, traj_p, sim_p)

% state renaming
q = x(1:end/2);
q_dot = x(end/2+1:end);

% trajectory sampling
xref = traj_p.sample_fun(t);

% control
u = control_p.control_fun(q, q_dot, xref, physics_p, control_p);

% dynamics
q_ddot = sim_p.dyn_fun(q, q_dot, u, physics_p);

% output build
x_dot = x;
x_dot(1:end/2) = q_dot;
x_dot(end/2+1:end) = q_ddot;

end