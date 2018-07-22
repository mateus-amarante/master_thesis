function [physics_p dyn_fun] = quadrotor2d_slung_physics()

% Physical parameters
physics_p.M = 0.18;
physics_p.m = physics_p.M*.8; % load mass
physics_p.L = 1; % cable length
physics_p.Itheta = 0.00025;
physics_p.g = 9.81;
physics_p.kt = 8.54858e-06;
physics_p.r = .086;
physics_p.maxThrust = 2*physics_p.M*physics_p.g;

% Dynamic model function
dyn_fun = @quadrotor2d_slung;
physics_p.dyn_fun = @quadrotor2d_slung;