function [physics_p, control_p, traj_p, sim_p, plot_p] = smc3d_slung_config()

    % Control parameters
    cparam = [10 1 10 1 7.536613519	1.88634197	17.94128065	0.547326511	9.247371112 -7.536613519	1.88634197	-17.94128065	0.547326511	9.247371112];
    control_p = common_control(cparam);    
    control_p.control_fun = @final_smc3d_slung_controller;
    
    [physics_p, traj_p, sim_p] = common_config();
    traj_p.sample_fun = waypoint_poly_trajectory(traj_p.td, [traj_p.rd, traj_p.rpy_d, traj_p.phithetaL_d ], 2);
    
    % Plot parameters
    plot_p.plot_state = @plot_quadrotor3d_slung_state;
    plot_p.plot_animation = @plot_quadrotor3d_slung_animation;
    
end