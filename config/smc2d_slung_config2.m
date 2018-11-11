function [physics_p, control_p, traj_p, plot_p] = smc2d_slung_config2()

% Physical parameters
physics_p = quadrotor2d_slung_physics();
physics_p.dyn_fun = @quadrotor2d_slung;

% Control parameters

cparam = [15.06485701	0.681483792	12.30158189	5.887881157	0.625333809	7.305426613];

control_p.lambda_z = cparam(1);
control_p.lambda_x = cparam(2);
control_p.lambda_theta = cparam(3);

control_p.kappa_z = cparam(4);
control_p.kappa_x = cparam(5);
control_p.kappa_theta = cparam(6);

control_p.eta_z = 0;
control_p.eta_x = 0;
control_p.eta_theta = 0;

control_p.n_input = 6;

control_p.control_fun = @smc2d_slung_controller2;

% Trajectory parameters
xd = 4;
zd = 2;
T = 3;
steady_time = 3;
dt = .01;
% traj_p = simple2d_shaped_trajectory(xd,zd,T,steady_time,dt,physics_p);
% traj_p = simple2d_trajectory(xd,zd,T,steady_time,dt);

% Trajectory parameters
xd = [0 0 0 1 1 1 1]';
zd = [0 0 0 1 1 1 1]';

pos = [xd zd];
vel = zeros(size(pos));
acc = zeros(size(pos));

traj_p.Tf = 12;
traj_p.dt = .02;
traj_p.t = 0:traj_p.dt:traj_p.Tf;

td = linspace(0,traj_p.Tf,length(xd));
q = [pos vel acc];
n_vars = size(pos,2);
n_deriv_out = 3;

% traj_p = simple3d_trajectory(xd,yd,zd,psid,T,steady_time,dt);
% traj_p = simple3d_shaped_trajectory(xd,yd,zd,psid,T,steady_time,dt,physics_p);
traj_p = shaped_poly_trajectory(td, q, n_vars, n_deriv_out, traj_p, physics_p);


traj_p.x0 = zeros(8,1);

% Plot parameters
plot_p.plot_state = @plot_quadrotor2d_slung_state;
plot_p.plot_animation = @plot_quadrotor2d_slung_animation;

end