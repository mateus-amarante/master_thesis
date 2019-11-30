function [physics_p, control_p, traj_p, sim_p, plot_p] = shaped_flat_smc3d_slung_config()
   
    % Control parameters
    control_p = common_control();
    control_p.control_fun = @final_smc3d_slung_controller;
    
    [physics_p, traj_p, sim_p] = common_config();
    [traj_p.sample_fun, traj_p.rL_fun, traj_p.yaw_fun] = waypoint_shaped_flat_trajectory(traj_p.td, traj_p.rLd, traj_p.psid, physics_p);
    sim_p.qd = traj_p.sample_fun(sim_p.t);

    % Plot parameters
    plot_p.plot_state = @plot_quadrotor3d_slung_flat_state;
    plot_p.plot_animation = @plot_quadrotor3d_slung_flat_animation;
    
end