function [physics_p] = quadrotor3d_slung_physics()

% Physical parameters
physics_p.M = 2.4;
physics_p.Ix = 0.055;
physics_p.Iy = 0.055;
physics_p.Iz = 0.1;
physics_p.I = diag([physics_p.Ix physics_p.Iy physics_p.Iz]);
physics_p.Iinv = inv(physics_p.I);
physics_p.g = 9.81;
physics_p.kt = 8.54858e-06;
physics_p.km = 1e-8;
physics_p.r = .65/2;
physics_p.maxThrust = 5*physics_p.M*physics_p.g;
physics_p.load_radius = .1;

physics_p.m = 1.0;
physics_p.L = 1.0;
physics_p.l = physics_p.L;

physics_p.cx = .1;
physics_p.cy = .1;
physics_p.cz = .1;
physics_p.Cd = [physics_p.cx physics_p.cy physics_p.cz]';
physics_p.cL = .05;

physics_p.wn = sqrt((physics_p.M+physics_p.m)*physics_p.g/(physics_p.M*physics_p.l));
physics_p.zeta = physics_p.cL/(2*physics_p.m*physics_p.wn);
physics_p.damping = sqrt(1-physics_p.zeta^2);
physics_p.wd = physics_p.damping*physics_p.wn;
physics_p.Td = 2*pi/physics_p.wd;
end