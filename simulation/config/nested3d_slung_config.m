function [physics_p control_p traj_p] = nested3d_slung_config()

% Physical parameters
physics_p.M = 0.18;
physics_p.Ix = 0.00025;
physics_p.Iy = 0.000232;
physics_p.Iz = 0.0003738;
physics_p.I = diag([physics_p.Ix physics_p.Iy physics_p.Iz]);
physics_p.Iinv = inv(physics_p.I);
physics_p.g = 9.81;
physics_p.kt = 4.5e-06;
physics_p.km = 1e-8;
physics_p.r = .086;
physics_p.maxThrust = 2*physics_p.M*physics_p.g;

physics_p.m = physics_p.M*.5;
physics_p.L = .4;

physics_p.dyn_fun = @quadrotor3d_slung;

% Control parameters
control_p.kp_xyz = [50 50 50]';
control_p.kd_xyz = [20 20 20]';

control_p.kp_rpy = [50 50 50]';
control_p.kd_rpy = [20 20 20]';

control_p.control_fun = @nested_control3d;
% control_p.control_fun = @(q,qdot,xref,physics_p,control_p) 314*ones(4,1);

% Trajectory parameters
traj_p.q0 = [0 0 0 0; 0 0 0 0; 0 0 0 0];
traj_p.qf = [1 1 1 1; 0 0 0 0; 0 0 0 0];
traj_p.tf = 2;
traj_p.tspan = [0 1.4*traj_p.tf]';
traj_p.traj_tspan = [0 traj_p.tf]';

traj_p.dt = .01;
traj_p.tt = traj_p.tspan(1):traj_p.dt:traj_p.tspan(2);

traj_p.x0 = zeros(16,1);
traj_p.x0(7) = .01;

traj_p.coef = plan_trajectory(traj_p.traj_tspan,traj_p.q0,traj_p.qf);

% traj_p.sample_fun = @(t) (t<traj_p.tf).*sample_trajectory(traj_p.coef,t) + ...
%     sample_trajectory(traj_p.coef,(t>=traj_p.tf)*traj_p.tf);
traj_p.sample_fun = @(t) my_sample_fun(t,traj_p.coef,traj_p.tf);

    function qd = my_sample_fun (t,coef,tf)
        t(t>=tf) = tf;
        qd = sample_trajectory(coef,t);
    end

end