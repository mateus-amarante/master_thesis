function [physics_p] = quadrotor3d_slung_physics()

physics_p.M_real = 2.4;
physics_p.m_real = 1;
physics_p.l_real = 1;

physics_p.Ix_real = 0.055;
physics_p.Iy_real =0.055;
physics_p.Iz_real = 0.1;
physics_p.I_real = diag([physics_p.Ix_real physics_p.Iy_real physics_p.Iz_real]);
physics_p.Iinv_real = inv(physics_p.I_real);

physics_p.cx_real = .2;
physics_p.cy_real = .2;
physics_p.cz_real = .5;
physics_p.cL_real = .1;

physics_p.default_uncertainty = 0.0;
physics_p.M_uncertainty = 0.0;
physics_p.m_uncertainty = 0.0;
physics_p.l_uncertainty = 0.0;
physics_p.I_uncertainty = physics_p.default_uncertainty;
physics_p.c_uncertainty = physics_p.default_uncertainty;

physics_p.M = (1+(rand()-.5)*physics_p.M_uncertainty)*physics_p.M_real;
physics_p.m = (1+(rand()-.5)*physics_p.m_uncertainty)*physics_p.m_real;
physics_p.l = (1+(rand()-.5)*physics_p.l_uncertainty)*physics_p.l_real;
physics_p.L = physics_p.l;
physics_p.I = (1+(rand(3)-.5)*physics_p.I_uncertainty).*physics_p.I_real;
physics_p.Iinv = inv(physics_p.I);
physics_p.Ix = physics_p.I(1,1);
physics_p.Iy = physics_p.I(2,2);
physics_p.Iz = physics_p.I(3,3);

physics_p.cx = (1+(rand()-.5)*physics_p.c_uncertainty)*physics_p.cx_real;
physics_p.cy = (1+(rand()-.5)*physics_p.c_uncertainty)*physics_p.cy_real;
physics_p.cz = (1+(rand()-.5)*physics_p.c_uncertainty)*physics_p.cz_real;
physics_p.cL = (1+(rand()-.5)*physics_p.c_uncertainty)*physics_p.cL_real;

% Physical parameters
% physics_p.M = 2.4;
% physics_p.Ix = 0.055;
% physics_p.Iy = 0.055;
% physics_p.Iz = 0.1;
% physics_p.I = diag([physics_p.Ix physics_p.Iy physics_p.Iz]);
% physics_p.Iinv = inv(physics_p.I);
physics_p.g = 9.81;
physics_p.kt = 8.54858e-06;
physics_p.km = 1e-8;
physics_p.r = .65/2;
physics_p.maxThrust = 5*physics_p.M*physics_p.g;
physics_p.load_radius = .1;

% physics_p.m = 1.0;
% physics_p.m = 5.0;
% physics_p.L = 1.0;
% physics_p.l = physics_p.L;

% physics_p.cx = .2;
% physics_p.cy = .2;
% physics_p.cz = .5;
% physics_p.cx = 0;
% physics_p.cy = 0;
% physics_p.cz = 0;
physics_p.Cd = [physics_p.cx physics_p.cy physics_p.cz]';
% physics_p.cL = .1;
% physics_p.cL = 0;

 
physics_p.wn = sqrt((physics_p.M+physics_p.m)*physics_p.g/(physics_p.M*physics_p.l));
% physics_p.zeta = physics_p.cL/(2*physics_p.m*physics_p.wn);
physics_p.zeta = 0;
physics_p.damping = sqrt(1-physics_p.zeta^2);
physics_p.wd = physics_p.damping*physics_p.wn;
physics_p.Td = 2*pi/physics_p.wd;
end