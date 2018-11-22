function [physics_p, control_p, traj_p, sim_p, plot_p] = smc2d_slung_config2()

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

% Trajectory and simulation parameters
xd = 5;
zd = 3;
T = 4;
wait_time = 1;
steady_time = 3;
dt = .02;

% traj_p = simple_smooth_trajectory([xd zd], T, wait_time, steady_time);
traj_p = simple_shaped_trajectory([xd zd], T, wait_time, steady_time, dt, physics_p);

sim_p.x0 = zeros(8, 1);
sim_p.dyn_fun = @quadrotor2d_slung;
sim_p.t = 0:dt:(wait_time + T + steady_time);

% Plot parameters
plot_p.plot_state = @plot_quadrotor2d_slung_state;
plot_p.plot_animation = @plot_quadrotor2d_slung_animation;

end