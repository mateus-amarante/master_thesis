function [physics_p, control_p, traj_p, sim_p, plot_p] = nested3d_slung_config()

% Physical parameters
physics_p = quadrotor3d_slung_physics();

% Control parameters
control_p.kp_xyz = [50 50 50]';
control_p.kd_xyz = [20 20 20]';

control_p.kp_rpy = [50 50 50]';
control_p.kd_rpy = [20 20 20]';

control_p.control_fun = @nested3d_controller;


% Trajectory parameters
xd = 2;
yd = 4;
zd = 3;
psid = -pi/6;
T = 4;
wait_time = 1;
steady_time = 3;
dt = .02;

% traj_p = simple_smooth_trajectory([xd zd], T, wait_time, steady_time);
traj_p = simple_shaped_trajectory([xd yd zd psid], T, wait_time, steady_time, dt, physics_p);

sim_p.x0 = zeros(12, 1);
sim_p.dyn_fun = @quadrotor3d;
sim_p.t = 0:dt:(wait_time + T + steady_time);


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

sim_p.x0 = zeros(16, 1);
sim_p.dyn_fun = @quadrotor3d_slung;
sim_p.t = 0:dt:(wait_time + T + steady_time);

% Plot parameters
plot_p.plot_state = @plot_quadrotor3d_slung_state;
plot_p.plot_animation = @plot_quadrotor3d_slung_animation;

end