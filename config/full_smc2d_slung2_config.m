function [physics_p, control_p, traj_p, sim_p, plot_p] = full_smc2d_slung2_config()

% Physical parameters
physics_p = quadrotor2d_slung_physics();

% Control parameters

cparam = [42.0112   49.9835   47.1140   41.8978    0.6964    0.5118   44.0564];




control_p.lambda_alpha = 0;
control_p.lambda_z = cparam(1);

control_p.lambda_x = cparam(2);
control_p.lambda_theta = cparam(3);
control_p.lambda_xdot = cparam(4);
control_p.lambda_thetadot = cparam(5);

control_p.kappa_z = cparam(6);
control_p.eta_z = 0;

control_p.kappa_xtheta = cparam(7);
control_p.eta_xtheta = 0;

control_p.K1 = 1;
control_p.K3 = 2;
control_p.K2 = control_p.lambda_xdot/control_p.lambda_x*control_p.K3;

disp([control_p.K1 control_p.K2 control_p.K3]);

% control_p.lambda_xdot = control_p.K2;
% control_p.lambda_x = control_p.K3;

roots([1, control_p.K1, control_p.K2, control_p.K3])

control_p.n_input = 7;

control_p.control_fun = @full_smc2d_slung_controller2;

% Trajectory and simulation parameters
xd = 1;
zd = 1;
T = 4;
wait_time = 1;
steady_time = 3;
dt = .02;

% traj_p = setpoint_trajectory([xd;zd;zeros(4,1)], wait_time + T + steady_time, dt);
traj_p = simple_smooth_trajectory([xd zd], T, wait_time, steady_time);
% traj_p = simple2d_shaped_trajectory(xd,zd, T, steady_time, dt, physics_p);
% traj_p = shaped_poly_trajectory([0;T;T+steady_time], [0,0;xd,zd;xd,zd], 2, 3, traj_p, physics_p);

sim_p.x0 = zeros(8, 1);
sim_p.x0(4) = pi/16;
sim_p.dyn_fun = @quadrotor2d_slung;
sim_p.t = 0:dt:(wait_time + T + steady_time);

% Plot parameters
plot_p.plot_state = @plot_quadrotor2d_slung_state;
plot_p.plot_animation = @plot_quadrotor2d_slung_animation;

end