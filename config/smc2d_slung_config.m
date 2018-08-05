function [physics_p control_p traj_p plot_p] = smc2d_slung_config()

% Physical parameters
physics_p = quadrotor2d_slung_physics();
physics_p.dyn_fun = @quadrotor2d_slung;

% Control parameters

cparam = [42.0112   49.9835   47.1140   41.8978    0.6964    0.5118   44.0564];

control_p.lambda_z = cparam(1);

control_p.lambda_x = cparam(2);
control_p.lambda_theta = cparam(3);
control_p.lambda_xdot = cparam(4);
control_p.lambda_thetadot = cparam(5);

control_p.kappa_z = cparam(6);
control_p.eta_z = 0;

control_p.kappa_xtheta = cparam(7);
control_p.eta_xtheta = 0;

control_p.n_input = 7;

control_p.control_fun = @smc2d_slung_controller;

% Trajectory parameters
xd = 3;
zd = -1;
T = 3;
steady_time = 5;
dt = .05;
traj_p = simple2d_shaped_trajectory(xd,zd,T,steady_time,dt,physics_p);
traj_p.x0 = zeros(8,1);

% Plot parameters
plot_p.plot_state = @plot_quadrotor2d_slung_state;
plot_p.plot_animation = @plot_quadrotor2d_slung_animation;

end