function plot_quadrotor2d_state(t, q, qdot, qd, u, physics_p, control_p)

%% State and reference remapping
x = q(:,1);
z = q(:,2);
theta = q(:,3);

xdot = qdot(:,1);
zdot = qdot(:,2);
thetadot = qdot(:,3);

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
plot(t, theta);
ylabel('Orientation $$\theta$$ [rad]');

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
plot(t,thetadot);
ylabel('Angular Velocity $$\dot{\theta}$$ [rad/s]');

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
end