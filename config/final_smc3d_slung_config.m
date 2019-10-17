function [physics_p, control_p, traj_p, sim_p, plot_p] = final_smc3d_slung_config()
   
    % Control parameters
    cparam = [2.73888001034958,9.38328717784594,25.4891504708899,7.25191305945691,10.7456362907187,49.3504117563270,23.1655698929931,1.48150742391072,21.5882927498974,-22.5283962226647,20.2333999834668,-49.7767880624373,1.56190934547965,44.4435957314630];

    control_p = common_control(cparam);
    control_p.control_fun = @final_smc3d_slung_controller;
    
    [physics_p, traj_p, sim_p] = common_config();
    traj_p.sample_fun = waypoint_flat_trajectory(traj_p.td, traj_p.rLd, traj_p.psid, physics_p);

    % Plot parameters
    plot_p.plot_state = @plot_quadrotor3d_slung_flat_state;
    plot_p.plot_animation = @plot_quadrotor3d_slung_flat_animation;
    
    end