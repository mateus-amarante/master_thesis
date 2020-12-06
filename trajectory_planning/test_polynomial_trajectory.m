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


% UP TO 6th derivative
x_d = [[3 0 2]' zeros(3, 7)];
sample_fun = plan_polynomial_trajectory2(t, x_d, 1, 2);


tt = t(1):.1:t(end);
qqd = sample_fun(tt);

figure;
plot(tt,qqd);
l = legend('Posição', 'Velocidade', 'Aceleração');
set(l, "Interpreter", 'tex');
xlim([min(tt) max(tt)]);
amp = max(max(qqd))-min(min(qqd));
ylim([min(min(qqd))-.1*amp, max(max(qqd))+.1*amp]);
xlabel("Tempo [s]");
ylabel("$q_d$");


% Simple 1D
sample_fun = plan_polynomial_trajectory2([0 4]', [0 0; 4 0], 1, 1);
tt = (0:.1:4)';
qqd = sample_fun(tt);

figure;
plot(tt,qqd);
l = legend('q(t)', 'v(t)');
grid on;
set(l, "FontSize", 16);
xlim([0 4]);
ylim([-.5, 4.5]);
xlabel("Tempo [s]");
