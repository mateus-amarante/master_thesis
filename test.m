clear all;
close all;
clc;

addpath('auxiliary','control','controllers','dynamics','physical_params',...
    'config','plot','trajectories','trajectory_planning');

xd = 5;
zd = 4;
T = 3;
steady_time = 3;
dt = .01;
traj_p = simple2d_trajectory(xd,zd,T,steady_time,dt);
plot(traj_p.qd(:,1))
hold on

M = .85;
m = .8*M;
g = 9.81;
L = 1;
wn = sqrt((M+m)*g/(M*L));
zeta = 0;
[u ts A] = zvd_shaper(traj_p.t,traj_p.qd,wn,zeta);
plot(traj_p.qd(:,1))
hold on
plot(u(:,1))
