% SIMPLE TEST OF plan_polynomial_trajectory function

t = [0 5 10]'; % Waypoints times
tt = (-2:.2:12)'; % Sample times

% Waypoints positions
x1_d = [0 3 1]'; x2_d = [3 0 2]';
x_d = [x1_d, x2_d];   

% Waypoints velocities and accelerations
xdot_d  = [0 0; 0 0; 0 0];
xddot_d = [0 0; 0 0; 0 0];

% LINEAR
sample_fun = plan_polynomial_trajectory(t, x_d);

qqd = sample_fun(tt);

figure;
plot(tt,qqd);
legend('pos1','pos2');

% CUBIC
sample_fun = plan_polynomial_trajectory(t, x_d, xdot_d);

qqd = sample_fun(tt);

figure;
plot(tt,qqd);
legend('pos1','pos2','vel1','vel2');

% QUINTIC
sample_fun = plan_polynomial_trajectory(t, x_d, xdot_d, xddot_d);

qqd = sample_fun(tt);

figure;
plot(tt,qqd);
legend('pos1','pos2','vel1','vel2','acc1','acc2');
