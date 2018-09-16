function [physics_p control_p traj_p plot_p] = smc2_config()

% Physical parameters
physics_p = quadrotor2d_slung_physics();
physics_p.dyn_fun = @quadrotor2d;

% Control parameters
control_p.lambda_z = 1;

control_p.lambda_x = 4;
control_p.lambda_theta = 2;
control_p.lambda_xdot = 1;
control_p.lambda_thetadot = .1;

control_p.kappa_z = 1;
control_p.eta_z = 0;

control_p.kappa_xtheta = 5;
control_p.eta_xtheta = 0;

control_p.n_input = 7;

control_p.control_fun = @smc2d_controller;

% Trajectory parameters
xd = 5;
zd = 3;
T = 3;
steady_time = 3;
dt = .05;
traj_p = simple2d_trajectory(xd,zd,T,steady_time,dt);

% Plot parameters
plot_p.plot_state = @plot_quadrotor2d_state;
plot_p.plot_animation = @plot_quadrotor2d_animation;

end