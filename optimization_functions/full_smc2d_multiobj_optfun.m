function F = full_smc2d_multiobj_optfun(x,physics_p,control_p,traj_p,sim_p)


control_p.beta_alpha = x(1);
control_p.gamma_z = x(2);

control_p.K_alpha = x(3);
control_p.K_alphadot = x(4);

try
    [t,x] = ode45(@(t, x) ode_fun(t, x, physics_p, control_p, traj_p, sim_p), sim_p.t, sim_p.x0);
catch ME
    disp(ME.identifier);
    F = [10^9; 10^9];
    return;
end

if length(t) < length(sim_p.t)
    F = [10^9; 10^9];
    return;
end

z = x(:,2);
alpha = x(:,4);

F(1) = z'*z;
F(2) = alpha'*alpha;


end