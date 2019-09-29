function [physics_p, dyn_fun] = quadrotor3d_slung_physics()

% Physical parameters
physics_p.M = 0.18;
physics_p.Ix = 0.00025;
physics_p.Iy = 0.000232;
physics_p.Iz = 0.0003738;
physics_p.I = diag([physics_p.Ix physics_p.Iy physics_p.Iz]);
physics_p.Iinv = inv(physics_p.I);
physics_p.g = 9.81;
physics_p.kt = 8.54858e-06;
physics_p.km = 1e-8;
physics_p.r = .15;
physics_p.maxThrust = 5*physics_p.M*physics_p.g;
physics_p.load_radius = .04;

physics_p.m = physics_p.M*.5;
physics_p.L = .4;
physics_p.l = physics_p.L;

physics_p.cx = .1;
physics_p.cy = .1;
physics_p.cz = .1;
physics_p.Cd = [physics_p.cx physics_p.cy physics_p.cz]';
physics_p.cL = .05;
