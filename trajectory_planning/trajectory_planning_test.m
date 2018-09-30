% SIMPLE TEST OF plan_trajectory and sample_trajectory functions

tspan = [0 10];
t = [tspan(1):.1:tspan(end)];

%LINEAR
q0=[0 1]; % Initial positions
qf=[1 0]; % Final positions

A = plan_trajectory(tspan,q0,qf); % Coeficient matrix
qd = sample_trajectory(A,t); % Desired trajectory

figure;
plot(t,qd);
legend('pos1','pos2','vel1','vel1','acc1','acc1');

% CUBIC
q0 = [0 0; 1 0]'; % Initial positions and velocities
qf = [1 0; 0 0]'; % Final positions and velocities

A = plan_trajectory(tspan,q0,qf);  % Coeficient matrix
qd = sample_trajectory(A,t); % Desired trajectory

figure;
plot(t,qd);
legend('pos1','pos2','vel1','vel1','acc1','acc1');

% QUINTIC
q0 = [0 0 0; 1 0 0]'; % Initial positions, velocities and accelerations
qf = [1 0 0; 0 0 0]'; % Final positions, velocities and accelerations

A = plan_trajectory(tspan,q0,qf); % Coeficient matrix
qd = sample_trajectory(A,t); % Desired trajectory

figure;
plot(t,qd);
legend('pos1','pos2','vel1','vel1','acc1','acc1');
