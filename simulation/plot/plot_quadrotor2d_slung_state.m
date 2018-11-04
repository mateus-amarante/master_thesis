function plot_quadrotor2d_slung_state(t,q,qdot,qd,u,physics_p, control_p)

% r = physics_p.r;
x = q(:,1);
z = q(:,2);
theta = q(:,3);
alpha = q(:,4);

xdot = qdot(:,1);
zdot = qdot(:,2);
thetadot = qdot(:,3);
alphadot = qdot(:,4);

xd = qd(:,1);
zd = qd(:,2);
xdot_d = qd(:,3);
zdot_d = qd(:,4);

%% Plot Robot Position and Orientation
figure;

subplot(2,1,1);
plot(t,[x z xd zd]);
legend('x','z','$$x_d$$','$$z_d$$');
ylabel('Position [m]');

subplot(2,1,2);
plot(t,[theta alpha]);
ylabel('Orientation [rad]');
legend('$$\theta$$','$$\alpha$$');

xlabel('Time [s]');
fig = gcf;
title(fig.Children(end), 'Robot Pose');

%% Plot Robot Lienar and Angular Velocities
figure;

subplot(2,1,1);
plot(t,[xdot zdot xdot_d zdot_d]);
legend('$$\dot{x}$$','$$\dot{z}$$','$$\dot{x}_d$$','$$\dot{z}_d$$');
ylabel('Linear Velocity [m/s]');

subplot(2,1,2);
plot(t,[thetadot alphadot]);
ylabel('Angular Velocity [rad/s]');
legend('$$\dot{\theta}$$','$$\dot{\alpha}$$');

xlabel('Time [s]');
fig = gcf;
title(fig.Children(end), 'Robot Twist');

%% Plot Control Input
figure;
subplot(2,1,1);
plot(t,u(:,1));
ylabel('$$U_1$$ [N]');

subplot(2,1,2);
plot(t,u(:,2));
ylabel('$$U_2$$ [N$$\cdot$$m]');
xlabel('Time [s]');

fig = gcf;
title(fig.Children(end), 'Control Input');

%% Plot sliding variables
for i=1:length(t)
    % FIXME: state variables are temporarily transposed for nested3d_control
    [~, ss] = control_p.control_fun(q(i,:)',qdot(i,:)',qd(i,:)',physics_p,control_p);
    s(:,i) = ss;
end

% sdot = diff(s')/(t(2)-t(1));
% plot(s(1,1:end-1),sdot(:,1));

figure;
plot(t, s.^2);
xlabel('Time [s]');
ylabel('$$s^2$$');
fig = gcf;
title(fig.Children(end), 'Lyapunov Function');
end