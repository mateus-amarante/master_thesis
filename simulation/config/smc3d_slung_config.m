function [physics_p, control_p, traj_p, plot_p] = smc3d_slung_config()

% Physical parameters
physics_p = quadrotor3d_slung_physics();
physics_p.dyn_fun = @quadrotor3d_slung;

% Control parameters
cparam = [42.0112   49.9835   47.1140   41.8978    0.6964    0.5118   44.0564];

control_p.lambda_zpsi = [cparam(1); cparam(1)];
control_p.kappa_zpsi = [cparam(6); cparam(6)];
control_p.eta_zpsi = zeros(2,1);

control_p.lambda_xtheta = [cparam(2); cparam(3)];
control_p.lambda_xtheta_dot = [cparam(4); cparam(5)];
control_p.kappa_xtheta = cparam(7);
control_p.eta_xtheta = 0;

control_p.lambda_yphi = [cparam(2); cparam(3)];
control_p.lambda_yphi_dot = [cparam(4); cparam(5)];
control_p.kappa_yphi = cparam(7);
control_p.eta_yphi = 0;

control_p.n_input = 14;

control_p.control_fun = @smc3d_slung_controller;

% Trajectory parameters
xd = [0 0 2 2 0 0 0]';
yd = [0 0 0 1 1 0 0]';
zd = [0 0 3 3 0 0 0]';
psid = zeros(size(xd));

pos = [xd yd zd psid];
vel = zeros(size(pos));
acc = zeros(size(pos));

traj_p.Tf = 10;
traj_p.dt = .02;
traj_p.t = 0:traj_p.dt:traj_p.Tf;

td = linspace(0,traj_p.Tf,length(xd));
q = [pos vel acc];
n_vars = size(pos,2);
n_deriv_out = 3;

% traj_p = simple3d_trajectory(xd,yd,zd,psid,T,steady_time,dt);
% traj_p = simple3d_shaped_trajectory(xd,yd,zd,psid,T,steady_time,dt,physics_p);
traj_p = shaped_poly_trajectory(td, q, n_vars, n_deriv_out, traj_p, physics_p);
% traj_p.sample_fun = plan_polynomial_trajectory(td, q, n_vars, n_deriv_out);
% traj_p.qd = traj_p.sample_fun(traj_p.t);

traj_p.x0 = zeros(16,1);

% Plot parameters
plot_p.plot_state = @plot_quadrotor3d_slung_state;
plot_p.plot_animation = @plot_quadrotor3d_slung_animation;

end