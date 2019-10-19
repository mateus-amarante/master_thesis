% SIMPLE TEST OF plan_polynomial_trajectory function

t = [0 5 10]'; % Waypoints times
tt = (-12:.01:12)'; % Sample times

% Waypoints positions
pos = [3 0;
       0 3;
       2 1];

vel = zeros(size(pos));

accel = vel;

testCase = matlab.unittest.TestCase.forInteractiveUse;

% LINEAR
x_d = pos;
sample_fun = plan_polynomial_trajectory2(t, x_d, 2, 2);

qqd = sample_fun(tt);

figure;
plot(tt,qqd);
legend('pos1', 'pos2', 'vel1', 'vel2', 'acc1', 'acc2');

% CUBIC
x_d = [pos vel];
sample_fun = plan_polynomial_trajectory2(t, x_d, 2, 2);
% sample_fun = plan_polynomial_trajectory(t, x_d, 2, 3);

qqd = sample_fun(tt);

figure;
plot(tt,qqd);
legend('pos1', 'pos2', 'vel1', 'vel2', 'acc1', 'acc2');

% QUINTIC
x_d = [pos vel accel];
sample_fun = plan_polynomial_trajectory2(t, x_d, 2, 2);
% sample_fun = plan_polynomial_trajectory(t, x_d, 2, 3);

qqd = sample_fun(tt);

figure;
plot(tt,qqd);
legend('pos1', 'pos2', 'vel1', 'vel2', 'acc1', 'acc2');