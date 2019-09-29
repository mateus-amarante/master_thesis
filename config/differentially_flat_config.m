function [physics_p, control_p, traj_p, sim_p, plot_p] = differentially_flat_config()

    % Physics
    physics_p = quadrotor3d_slung_physics();
    
    control_p.control_fun = @differentially_flat_open_loop_controller;
    control_p.n_inputs = 4;
    
    l = physics_p.l;
    
    % Trajectory and simulation parameters
    xLd = [ 0  0 2 2 0 0 0]';
    yLd = [ 0  0 0 1 1 0 0]';
    zLd = [-l -l 1 1 1 -l -l]';
    rLd= [xLd yLd zLd];
    
    psid = [0 0 0 pi/2 pi/2 0 0]';
    
    Tf = 20;
    td = linspace(0,Tf,length(xLd))';
    
    sim_p.x0 = zeros(16, 1);
    sim_p.dyn_fun = @quadrotor_slung_load_3d;
    sim_p.t = 0:.02:Tf;

    traj_p.sample_fun = waypoint_flat_trajectory(td, rLd, psid, physics_p);
       
    % Plot parameters
    plot_p.plot_state = @plot_quadrotor3d_slung_flat_state;
    plot_p.plot_animation = @plot_quadrotor3d_slung_flat_animation;
    
end