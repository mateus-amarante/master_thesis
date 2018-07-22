function [physics_p control_p traj_p plot_p] = nested2d_slung_config()

% Physical parameters
physics_p = quadrotor2d_slung_physics();
physics_p.dyn_fun = @quadrotor2d_slung;

% Control parameters
control_p.kp_x = 50;
control_p.kd_x = 10;

control_p.kp_z = 50;
control_p.kd_z = 10;

control_p.kp_theta = 80;
control_p.kd_theta = 20;

control_p.control_fun = @nested2d_controller;

% Trajectory parameters
xd = 5;
zd = 4;
T = 3;
steady_time = 3;
dt = .05;
traj_p = simple2d_trajectory(xd,zd,T,steady_time,dt);
traj_p.x0 = zeros(8,1);

% Plot parameters
plot_p.plot_state = @plot_quadrotor2d_slung_state;
plot_p.plot_animation = @plot_quadrotor2d_slung_animation;

end