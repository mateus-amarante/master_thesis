startup;

% PICK YOUR CONFIG FUNCTION
% [physics_p, control_p, traj_p, sim_p, plot_p] = flat_smc3d_slung_config();
[physics_p, control_p, traj_p, sim_p, plot_p] = shaped_flat_smc3d_slung_config();
% [physics_p, control_p, traj_p, sim_p, plot_p] = smc3d_slung_config();
% [physics_p, control_p, traj_p, sim_p, plot_p] = shaped_smc3d_slung_config();
% [physics_p, control_p, traj_p, sim_p, plot_p] = differentially_flat_config();  NOT WORKING

[t,x,qd,u,metrics,s,ueq,usw] = run_simulation(physics_p, control_p, traj_p, sim_p);

plot_p.plot_state(t, x, physics_p, control_p, traj_p, plot_dictionary('en'));
plot_p.plot_animation(t, x, physics_p, traj_p);
