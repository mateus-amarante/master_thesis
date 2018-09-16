function plot_quadrotor3d_state(t,q,qdot,qd,u,physics_p)

% Robot Position
x = q(:,1);
y = q(:,2);
z = q(:,3);

phi = q(:,4);
theta = q(:,5);
psi = q(:,6);

% Robot Velocity
xdot = qdot(:,1);
ydot = qdot(:,2);
zdot = qdot(:,3);

phidot = qdot(:,4);
thetadot = qdot(:,5);
psidot = qdot(:,6);

% Desired State
xd = qd(:,1);
yd = qd(:,2);
zd = qd(:,3);
psid = qd(:,4);

xdot_d = qd(:,5);
ydot_d = qd(:,6);
zdot_d = qd(:,7);
psidot_d = qd(:,8);

% Plot
figure;
plot(t,[x y z phi theta psi],t,[xd yd zd psid]);
legend('x','y','z','$$\phi$$','$$\theta$$','$$\psi$$','$$x_d$$','$$y_d$$','$$z_d$$','$$\psi_d$$');
title('Full State + Full Desired Trajectory');
xlabel('Time [s]');

% figure;
% plot(t,[xdot zdot],t,[xdot_d zdot_d]);
% legend('x\dot','z\dot','x\dot_d','z\dot_d');
% title('Velocities');
% xlabel('Time [s]');

figure;
plot(t,u);
legend('$$U_1$$','$$U_2$$','$$U_3$$','$$U_4$$');
title('Control Input');
xlabel('Time [s]');
end