function traj_p = simple3d_trajectory(x,y,z,psi,T,steady_time,dt)

% Trajectory parameters
traj_p.q0 = [0 0 0 0; 0 0 0 0; 0 0 0 0];
traj_p.qf = [x y z psi; 0 0 0 0; 0 0 0 0];
traj_p.Tf = T + steady_time;
traj_p.traj_tspan = [0 T]';
% traj_p.traj_t = traj_p.tspan(1):traj_p.dt:traj_p.tspan(2);

% Simulation parameters
traj_p.tspan = [0 traj_p.Tf]';
traj_p.dt = dt;
traj_p.t = (0:dt:traj_p.Tf)';
traj_p.x0 = zeros(12,1);

% Trajectory building
traj_p.coef = plan_trajectory(traj_p.traj_tspan,traj_p.q0,traj_p.qf);

traj_p.sample_fun = @(t) my_sample_fun(t,traj_p.coef,T);
traj_p.qd = traj_p.sample_fun(traj_p.t)';

    function qd = my_sample_fun (t,coef,T)
        t(t>=T) = T;
        qd = sample_trajectory(coef,t);
    end

end

