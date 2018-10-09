function [physics_p, control_p, traj_p, plot_p] = nested3d_slung_config()

% Physical parameters
physics_p = quadrotor3d_slung_physics();
physics_p.dyn_fun = @quadrotor3d_slung;

% Control parameters
control_p.kp_xyz = [50 50 50]';
control_p.kd_xyz = [20 20 20]';

control_p.kp_rpy = [50 50 50]';
control_p.kd_rpy = [20 20 20]';

control_p.control_fun = @nested3d_controller;

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
traj_p.x0 = zeros(16,1);

% Plot parameters
plot_p.plot_state = @plot_quadrotor3d_slung_state;
plot_p.plot_animation = @plot_quadrotor3d_slung_animation;

end