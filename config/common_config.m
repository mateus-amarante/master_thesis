function [physics_p, traj_p, sim_p] = common_config()

    % Physical parameters
    physics_p = quadrotor3d_slung_physics();

    % Trajectory and simulation parameters
    traj_p.xd = [0 4 4 1 0]';
    traj_p.yd = [0 6 6 9 0]';
    traj_p.zd = [0 5 5 2 0]';
    traj_p.psid = [0 pi/3 pi/3 -pi/4 0]';
        
    traj_p.Tf = 15;cd 
    traj_p.td = linspace(0,traj_p.Tf,length(traj_p.xd))';
    traj_p.wait_time = 3;
    traj_p.setling_time= 5;
    
    traj_p.td = [-traj_p.wait_time; traj_p.td; traj_p.td(end)+traj_p.setling_time] + traj_p.wait_time;
    traj_p.xd = [traj_p.xd(1); traj_p.xd; traj_p.xd(end)];
    traj_p.yd = [traj_p.yd(1); traj_p.yd; traj_p.yd(end)];
    traj_p.zd = [traj_p.zd(1); traj_p.zd; traj_p.zd(end)];
    traj_p.psid = [traj_p.psid(1); traj_p.psid; traj_p.psid(end)];

    traj_p.stop_time = traj_p.td(end-1);
    traj_p.Tf = traj_p.td(end);

%     % Config diff_flat_shaped
%     traj_p.xd = [0 0 5 5]';
%     traj_p.yd = [0 0 5 5]'
%     traj_p.zd = [0 0 5 5]';
%     traj_p.psid = [0 0 pi/3 pi/3]';
%     
%     traj_p.Tf = 15;
%     traj_p.td = [0 2 10 15]';
%     traj_p.Tf = traj_p.td(end);
%     
    
  
    traj_p.rd = [traj_p.xd traj_p.yd traj_p.zd];
    traj_p.zLd = traj_p.zd - physics_p.l;
    traj_p.rLd = [traj_p.xd traj_p.yd traj_p.zLd];
    
    traj_p.rpy_d = [zeros(length(traj_p.psid),2), traj_p.psid];
    traj_p.phithetaL_d = zeros(length(traj_p.psid),2);
    
    
    sim_p.x0 = [traj_p.rd(1, :), traj_p.rpy_d(1, :), traj_p.phithetaL_d(1,:), zeros(1,8) ]';
%     sim_p.x0(7:8) = deg2rad(10);
%     sim_p.x0(1:3) = .1;
%     sim_p.x0(4:6) = deg2rad(10);

    sim_p.dyn_fun = @quadrotor_slung_load_3d;
    sim_p.dt = 0.01;
    sim_p.t = (0:sim_p.dt:traj_p.Tf)';
    sim_p.Fs = 1/sim_p.dt;
    sim_p.N = length(sim_p.t)-1;
    sim_p.f = sim_p.Fs/sim_p.N*(0:sim_p.N/2);

%     traj_p.sample_fun = waypoint_poly_trajectory(td, [rd, rpy_d, rpL_d], 2);
    
    noise_freq = 100;
    noise_t = 0:1/noise_freq:traj_p.Tf;
 
    xy_error = normrnd(0, .005, [2, numel(noise_t)]);
    z_error = normrnd(0, .005, [1, numel(noise_t)]);
    
    xydot_error = normrnd(0, .01, [2, numel(noise_t)]);
    zdot_error = normrnd(0, .01, [1, numel(noise_t)]);
    
    phitheta_error = normrnd(0, deg2rad(.1), [2, numel(noise_t)]);
    psi_error = normrnd(0, deg2rad(.1), [1, numel(noise_t)]);
    
    phithetadot_error = normrnd(0, deg2rad(.1), [2, numel(noise_t)]);
    psidot_error = normrnd(0, deg2rad(.1), [1, numel(noise_t)]);
    
    phithetaL_error = normrnd(0, deg2rad(.1), [2, numel(noise_t)]);
    phithetaLdot_error = normrnd(0, deg2rad(.1), [2, numel(noise_t)]);
    
    position_error = [xy_error; z_error; phitheta_error; psi_error; phithetaL_error];
    velocity_error = [xydot_error; zdot_error; phithetadot_error; psidot_error; phithetaLdot_error];
    
%     p = pchip(noise_t, [position_error; velocity_error]);
%     sim_p.noise = @(t) ppval(p, t);
%     sim_p.noise = @(t) interp1(noise_t', [position_error; velocity_error]', t,'nearest')*.2;
    sim_p.noise = @(t) zeros(size(t));
    
    % Plot parameters
%     plot_p.plot_state = @plot_quadrotor3d_slung_flat_state;
%     plot_p.plot_animation = @plot_quadrotor3d_slung_animation;
    
    end