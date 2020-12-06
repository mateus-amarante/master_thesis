function [t,x,qd,u,metrics,s,ueq,usw] = run_simulation(physics_p, control_p, traj_p, sim_p)
[t,x] = ode45(@(t, x) ode_fun(t, x, physics_p, control_p, traj_p, sim_p), sim_p.t, sim_p.x0);

%% Input trajectory
qd = traj_p.sample_fun(t);

%% Control input
u = zeros(length(t), control_p.n_inputs);
s = u;
ueq = u;
usw = u;

for i=1:length(t)
    [u(i,:), s(i,:), ueq(i,:), usw(i,:)] = control_p.control_fun(x(i,:),qd,physics_p,control_p);
end

metrics = calc_metrics(t,x,qd,u,sim_p.Fs,traj_p.stop_time);

end

