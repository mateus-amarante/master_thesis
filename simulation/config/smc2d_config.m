function [physics_p, control_p, traj_p, sim_p, plot_p] = smc2d_config()

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

% Trajectory and simulation parameters
xd = 5;
zd = 3;
T = 4;
wait_time = 1;
steady_time = 3;
dt = .02;

traj_p = simple_smooth_trajectory([xd zd], T, wait_time, steady_time);

sim_p.x0 = zeros(6, 1);
sim_p.dyn_fun = @quadrotor2d;
sim_p.t = 0:dt:(wait_time + T + steady_time);

% Plot parameters
plot_p.plot_state = @plot_quadrotor2d_state;
plot_p.plot_animation = @plot_quadrotor2d_animation;

end