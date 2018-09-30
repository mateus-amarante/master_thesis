function plot_quadrotor2d_state(t,q,qdot,qd,u,physics_p)

x = q(:,1);
z = q(:,2);
theta = q(:,3);

% xdot = qdot(:,1);
% zdot = qdot(:,2);

% xdot_d = qd(:,3);
% zdot_d = qd(:,4);

xd = qd(:,1);
zd = qd(:,2);

%% Plot Robot State
figure;
subplot(2,1,1);
plot(t,[x z xd zd]);
ylabel('Position [m]'); %xlabel('Time [s]');
legend('x','z','$$x_d$$','$$z_d$$');

subplot(2,1,2);
plot(t,theta);
ylabel('$$\theta$$ [rad]'); xlabel('Time [s]');

fig = gcf;
title(fig.Children(end), 'Robot State');

% 
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