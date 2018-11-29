function plot_quadrotor3d_state(t,q,qdot,qd,u,physics_p,control_p)

% Robot Position
x = q(:,1);
y = q(:,2);
z = q(:,3);

phi = q(:,4);
theta = q(:,5);
psi = q(:,6);

% Robot Velocity
% xdot = qdot(:,1);
% ydot = qdot(:,2);
% zdot = qdot(:,3);
% 
% phidot = qdot(:,4);
% thetadot = qdot(:,5);
% psidot = qdot(:,6);

% Desired State
xd = qd(:,1);
yd = qd(:,2);
zd = qd(:,3);
psid = qd(:,4);

% xdot_d = qd(:,5);
% ydot_d = qd(:,6);
% zdot_d = qd(:,7);
% psidot_d = qd(:,8);


%% Plot Robot State
figure;
subplot(2,1,1);
plot(t,[x y z xd yd zd]);
ylabel('Position [m]'); %xlabel('Time [s]');
legend('x','y','z','$$x_d$$','$$y_d$$','$$z_d$$');

subplot(2,1,2);
plot(t,[phi theta psi psid]);
legend('$$\phi$$','$$\theta$$','$$\psi$$','$$\psi_d$$');
ylabel('Orientation [rad]');

xlabel('Time [s]');
fig = gcf;
title(fig.Children(end), 'Robot State');


%% Plot Control Input
figure;
subplot(2,1,1);
plot(t,u(:,1));
ylabel('Thrust Force $$U_1$$ [N]');

subplot(2,1,2);
plot(t,u(:,2:end));
ylabel('Input Torque [N$$\cdot$$m]');
legend('$$U_2$$','$$U_3$$','$$U_4$$');

xlabel('Time [s]');
fig = gcf;
title(fig.Children(end), 'Control Input');

end