function [physics_p, control_p, traj_p, plot_p] = nested2d_config()

% Physical parameters
physics_p = quadrotor2d_slung_physics();
physics_p.dyn_fun = @quadrotor2d;

% Control parameters
control_p.kp_x = 50;
control_p.kd_x = 10;

control_p.kp_z = 50;
control_p.kd_z = 10;

control_p.kp_theta = 80;
control_p.kd_theta = 20;

control_p.n_input = 6;

control_p.control_fun = @nested2d_controller;

% Trajectory parameters
xd = 5;
zd = 3;
T = 4;
steady_time = 3;
dt = .01;
traj_p = simple2d_trajectory(xd,zd,T,steady_time,dt);

% Plot parameters
plot_p.plot_state = @plot_quadrotor2d_state;
plot_p.plot_animation = @plot_quadrotor2d_animation;

end