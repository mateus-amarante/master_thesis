startup

% PICK YOUR CONFIG FUNCTION
% [physics_p, control_p, traj_p, plot_p] = nested2d_config();
% [physics_p, control_p, traj_p, plot_p] = smc2d_config();
% [physics_p, control_p, traj_p, plot_p] = nested2d_slung_config();
% [physics_p, control_p, traj_p, plot_p] = smc2d_slung_config();
[physics_p, control_p, traj_p, plot_p] = smc3d_slung_config();

% [physics_p, control_p, traj_p, plot_p] = nested3d_config();
% [physics_p, control_p, traj_p, plot_p] = nested3d_slung_config();

[t,x] = ode15s(@(t,x) ode_fun(t,x,physics_p,control_p,traj_p), traj_p.t, traj_p.x0);

qd = traj_p.qd;%traj_p.sample_fun(t)'; % TODO: build trajectory before (in traj_p construction)
q = x(:,1:end/2);
q_dot = x(:,end/2+1:end);

% TODO: define u for all timespecs at once
for i=1:length(t)
    % FIXME: state variables are temporarily transposed for nested3d_control
    u(i,:) = control_p.control_fun(q(i,:)',q_dot(i,:)',qd(i,:)',physics_p,control_p)';
end

plot_p.plot_state(t,q,q_dot,qd,u,physics_p);
plot_p.plot_animation(t,q,qd,physics_p);
