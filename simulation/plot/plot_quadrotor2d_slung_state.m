function plot_quadrotor2d_slung_state(t,q,qdot,qd,u,physics_p)

% r = physics_p.r;
x = q(:,1);
z = q(:,2);
theta = q(:,3);
alpha = q(:,4);

% xdot = qdot(:,1);
% zdot = qdot(:,2);

xd = qd(:,1);
zd = qd(:,2);

% xdot_d = qd(:,3);
% zdot_d = qd(:,4);

%% Plot Robot State
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
title(fig.Children(end), 'Robot State');

% figure;
% plot(t,[xdot zdot],t,[xdot_d zdot_d]);
% legend('$$\dot{x}$$','$$\dot{z}$$','$$\dot{x}_d$$','$$\dot{z}_d$$');
% title('Velocities');
% xlabel('Time [s]');

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