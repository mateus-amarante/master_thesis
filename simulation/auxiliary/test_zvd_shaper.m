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

v = sample_fun(t);
t = 0:dt:T+steady_time;


physics_p = quadrotor3d_slung_physics();

wn = physics_p.wn;
zeta = physics_p.zeta;
[u, ts, A] = zvd_shaper(t,v(:,1),wn,zeta);
plot(t,v(:,1),t,u);
% hold on
% plot(u)
legend('Original', 'Modulado')
xlabel('Tempo [s]');
% hold off;
figure;
stem(ts,A, 'LineWidth', 2);
xlabel('Tempo [s]');
ylim([0,0.6])
xlim([t(1) t(end)]);
