t = [0 5 10]'; % Waypoints times
tt = (-2:.01:12)'; % Sample times

% Waypoints positions
pos = [3 0 0;
       0 3 0;
       2 1 0];

vel = zeros(size(pos));
accel = vel;
jerk = vel;
snap = vel;

x_d = [pos vel accel];
sample_fun = plan_polynomial_trajectory(t, x_d, 3, 7);

qqd = sample_fun(tt);

% figure;
% plot(tt,qqd);

physics_p = quadrotor3d_slung_physics();


[flat_outputs] = differentially_flat_trajectory(qqd, zeros(length(tt), 3), physics_p);

% plot(tt, qqd(:,7:9), tt, flat_outputs(:,7:9));