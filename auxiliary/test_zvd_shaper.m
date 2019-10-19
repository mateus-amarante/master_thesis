% clear all;
close all;
clc;

addpath('auxiliary','control','controllers','dynamics','physics',...
    'config','plot','trajectories','trajectory_planning');

xd = 5;
zd = 4;
T = 3;
steady_time = 3;
dt = .01;
sample_fun = waypoint_poly_trajectory([0;T], [0;xd], 2);

t = 0:dt:T+steady_time;
plot(sample_fun(t));
hold on

M = .85;
m = .8*M;
g = 9.81;
L = 1;
wn = sqrt((M+m)*g/(M*L));
zeta = 0;
[u, ts, A] = zvd_shaper(t,sample_fun(t),wn,zeta);
plot(sample_fun(t));
hold on
plot(u)
