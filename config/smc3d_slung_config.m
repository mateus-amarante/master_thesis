function [physics_p, control_p, traj_p, plot_p] = smc3d_slung_config()

% Physical parameters
physics_p = quadrotor3d_slung_physics();
physics_p.dyn_fun = @quadrotor3d_slung;

% Control parameters
cparam = [1 1 1 1 7.536613519	1.88634197	17.94128065	0.547326511	9.247371112 7.536613519	1.88634197	17.94128065	0.547326511	9.247371112];
% cparam = [1 1 1 1 7.096489323	20.95267537	15.55696503	1.302720091	10.0564969 7.096489323	20.95267537	15.55696503	1.302720091	10.0564969];


control_p.lambda_zpsi = [cparam(1); cparam(2)];
control_p.kappa_zpsi = [cparam(3); cparam(4)];
control_p.eta_zpsi = zeros(2,1);

control_p.lambda_xtheta = [cparam(5); cparam(6)];
control_p.lambda_xtheta_dot = [cparam(7); cparam(8)];
control_p.kappa_xtheta = cparam(9);
control_p.eta_xtheta = 0;

control_p.lambda_yphi = [-cparam(10); cparam(11)];
control_p.lambda_yphi_dot = [-cparam(12); cparam(13)];
control_p.kappa_yphi = cparam(14);
control_p.eta_yphi = 0;

control_p.n_input = 5;

control_p.control_fun = @smc3d_slung_controller;

% Trajectory parameters
xd = [0 0 2 2 0 0 0]';
yd = [0 0 0 1 1 0 0]';
zd = [0 0 1 1 0 0 0]';
psid = zeros(size(xd));
psid = [0 0 0 pi/2 pi/2 pi pi]';

pos = [xd yd zd psid];
vel = zeros(size(pos));
acc = zeros(size(pos));

traj_p.Tf = 12;
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
% traj_p.x0(6) = pi/2;

% Plot parameters
plot_p.plot_state = @plot_quadrotor3d_slung_state;
plot_p.plot_animation = @plot_quadrotor3d_slung_animation;

end