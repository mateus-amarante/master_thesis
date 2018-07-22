function traj_p = simple2d_trajectory(x,z,T,steady_time,dt)

% Trajectory parameters
traj_p.q0 = [0  0; 0 0; 0 0];
traj_p.qf = [x z; 0 0; 0 0];
traj_p.Tf = T + steady_time;
traj_p.tspan = [0 traj_p.Tf]';
traj_p.traj_tspan = [0 T]';

traj_p.dt = dt;
traj_p.tt = traj_p.tspan(1):traj_p.dt:traj_p.tspan(2);

traj_p.x0 = zeros(6,1);

traj_p.coef = plan_trajectory(traj_p.traj_tspan,traj_p.q0,traj_p.qf);

% traj_p.sample_fun = @(t) (t<traj_p.tf).*sample_trajectory(traj_p.coef,t) + ...
%     sample_trajectory(traj_p.coef,(t>=traj_p.tf)*traj_p.tf);
traj_p.sample_fun = @(t) my_sample_fun(t,traj_p.coef,T);

    function qd = my_sample_fun (t,coef,T)
        t(t>=T) = T;
        qd = sample_trajectory(coef,t);
    end

end

