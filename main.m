clear all;
close all;
clc;

addpath('auxiliary','control','controllers','dynamics','physical_params',...
    'config','plot','trajectories','trajectory_planning');

% Pick your config function
[physics_p control_p traj_p plot_p] = nested2d_config();

[t,x] = ode45(@(t,x) ode_fun(t,x,physics_p,control_p,traj_p), traj_p.tt, traj_p.x0);

qd = traj_p.sample_fun(t)'; % TODO: build trajectory before (in traj_p construction)
q = x(:,1:end/2);
q_dot = x(:,end/2+1:end);

% TODO: define u for all timespecs at once
for i=1:length(t)
    u(i,1:2) = control_p.control_fun(q(i,:),q_dot(i,:),qd(i,:),physics_p,control_p)';
end

plot_p.plot_state(t,q,qd,u,physics_p);
plot_p.plot_animation(t,q,qd,physics_p);
