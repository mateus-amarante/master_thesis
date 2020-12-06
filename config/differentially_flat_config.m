function [physics_p, control_p, traj_p, sim_p, plot_p] = differentially_flat_config()

    % Control parameters
    control_p.control_fun = @differentially_flat_open_loop_controller;
    control_p.n_inputs = 4;
    
    [physics_p, traj_p, sim_p] = common_config();
    traj_p.sample_fun = waypoint_flat_trajectory(traj_p.td, traj_p.rLd, traj_p.psid, physics_p);
    traj_p.load_sample_fun = waypoint_poly_trajectory(traj_p.td, traj_p.rLd, traj_p.psid, physics_p);

    % Plot parameters
    plot_p.plot_state = @plot_quadrotor3d_slung_flat_state;
    plot_p.plot_animation = @plot_quadrotor3d_slung_flat_animation;
     
end