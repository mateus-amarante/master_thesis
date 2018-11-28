function [physics_p, control_p, traj_p, sim_p, plot_p] = nested2d_slung_config()

% Physical parameters
physics_p = quadrotor2d_slung_physics();

% Control parameters
% 73.5951   87.9026   99.7530   84.6533   99.5703   13.5317


control_p.kp_x = 73.5951;
control_p.kd_x = 87.9026;

control_p.kp_z = 99.7530;
control_p.kd_z = 84.6533;

control_p.kp_theta = 99.5703;
control_p.kd_theta = 13.5317;

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

sim_p.x0 = zeros(8, 1);
sim_p.dyn_fun = @quadrotor2d_slung;
sim_p.t = 0:dt:(wait_time + T + steady_time);

% Plot parameters
plot_p.plot_state = @plot_quadrotor2d_slung_state;
plot_p.plot_animation = @plot_quadrotor2d_slung_animation;

end