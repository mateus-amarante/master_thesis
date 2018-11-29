function [physics_p, control_p, traj_p, sim_p, plot_p] = nested2d_config()

% Physical parameters
physics_p = quadrotor2d_slung_physics();

% Control parameters
control_p.kp_x = 50;
control_p.kd_x = 10;

control_p.kp_z = 50;
control_p.kd_z = 10;

control_p.kp_theta = 80;
control_p.kd_theta = 20;

control_p.n_input = 6;

control_p.control_fun = @nested2d_controller;

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