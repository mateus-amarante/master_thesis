function [physics_p, traj_p, sim_p] = common_config()

    % Physical parameters
    physics_p = quadrotor3d_slung_physics();

    % Trajectory and simulation parameters
    traj_p.xd = [0 0 0 3 3 3]';
    traj_p.yd = [0 0 0 3 3 3]';
    traj_p.zd = [0 0 0 3 3 3]';
    traj_p.zLd = traj_p.zd - physics_p.l;

    traj_p.rd = [traj_p.xd traj_p.yd traj_p.zd];
    traj_p.rLd = [traj_p.xd traj_p.yd traj_p.zLd];
    
    traj_p.psid = [0 0 0 pi/4 pi/4 pi/4]';
    traj_p.rpy_d = [zeros(length(traj_p.psid),2), traj_p.psid];
    traj_p.phithetaL_d = zeros(length(traj_p.psid),2);
        
    traj_p.Tf = 12;
    traj_p.td = linspace(0,traj_p.Tf,length(traj_p.xd))';
    
    sim_p.x0 = [traj_p.rd(1, :), traj_p.rpy_d(1, :), traj_p.phithetaL_d(1,:), zeros(1,8) ]';
    
    sim_p.dyn_fun = @quadrotor_slung_load_3d;
    sim_p.dt = .02;
    sim_p.t = 0:sim_p.dt:traj_p.Tf;

%     traj_p.sample_fun = waypoint_poly_trajectory(td, [rd, rpy_d, rpL_d], 2);
    
    noise_freq = 100;
    noise_t = 0:1/noise_freq:traj_p.Tf;
 
    xy_error = normrnd(0, .01, [2, numel(noise_t)]);
    z_error = normrnd(0, .01, [1, numel(noise_t)]);
    
    xydot_error = normrnd(0, .01, [2, numel(noise_t)]);
    zdot_error = normrnd(0, .01, [1, numel(noise_t)]);
    
    phitheta_error = normrnd(0, .1*pi/180, [2, numel(noise_t)]);
    psi_error = normrnd(0, .1*pi/180, [1, numel(noise_t)]);
    
    phithetadot_error = normrnd(0, .1*pi/180, [2, numel(noise_t)]);
    psidot_error = normrnd(0, .1*pi/180, [1, numel(noise_t)]);
    
    phithetaL_error = normrnd(0, .1*pi/180, [2, numel(noise_t)]);
    phithetaLdot_error = normrnd(0, .1*pi/180, [2, numel(noise_t)]);
    
    position_error = [xy_error; z_error; phitheta_error; psi_error; phithetaL_error];
    velocity_error = [xydot_error; zdot_error; phithetadot_error; psidot_error; phithetaLdot_error];
    
    p = pchip(noise_t, [position_error; velocity_error]);
    sim_p.noise = @(t) ppval(p, t);
%     sim_p.noise = @(t) zeros(size(t));
    
    % Plot parameters
%     plot_p.plot_state = @plot_quadrotor3d_slung_flat_state;
%     plot_p.plot_animation = @plot_quadrotor3d_slung_animation;
    
    end