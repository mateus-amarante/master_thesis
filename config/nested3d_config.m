function [physics_p control_p traj_p plot_p] = nested3d_config()

% Physical parameters
physics_p = quadrotor3d_slung_physics();
physics_p.dyn_fun = @quadrotor3d;

% Control parameters
control_p.kp_xyz = [50 50 50]';
control_p.kd_xyz = [20 20 20]';

control_p.kp_rpy = [50 50 50]';
control_p.kd_rpy = [20 20 20]';

control_p.control_fun = @nested3d_controller;

% Trajectory parameters
xd = 2;
yd = 1;
zd = 3;
psid = -pi/6;
T = 3;
steady_time = 2;
dt = .01;
traj_p = simple3d_trajectory(xd,yd,zd,psid,T,steady_time,dt);

% Plot parameters
plot_p.plot_state = @plot_quadrotor3d_state;
plot_p.plot_animation = @plot_quadrotor3d_animation;

end