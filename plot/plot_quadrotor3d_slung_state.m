function plot_quadrotor3d_slung_state(t,q,physics_p, control_p, traj_p)

% State remapping
x = q;
q = x(:,1:end/2);
qdot = x(:,end/2+1:end);

% Compute desired trajectory
qd = traj_p.sample_fun(t);

u = zeros(length(t), control_p.n_inputs);
s = u;
ueq = u;
usw = u;

% TODO: define u for all timespecs at once
for i=1:length(t)
    [u(i,:), s(i,:), ueq(i,:), usw(i,:)] = control_p.control_fun(x(i,:),qd(i,:),physics_p,control_p);
end

% Robot Position
x = q(:,1);
y = q(:,2);
z = q(:,3);

phi = q(:,4);
theta = q(:,5);
psi = q(:,6);

phiL = q(:,7);
thetaL = q(:,8);

% Robot Velocity
xdot = qdot(:,1);
ydot = qdot(:,2);
zdot = qdot(:,3);

phidot = qdot(:,4);
thetadot = qdot(:,5);
psidot = qdot(:,6);

phiLdot = qdot(:,7);
thetaLdot = qdot(:,8);

% Desired State
xd = qd(:,1);
yd = qd(:,2);
zd = qd(:,3);
psid = qd(:,6);

xdot_d = qd(:,9);
ydot_d = qd(:,10);
zdot_d = qd(:,11);
psidot_d = qd(:,14);


%% Plot Robot State
figure;
subplot(2,1,1);
plot(t,[x y z xd(1:length(x)) yd(1:length(y)) zd(1:length(z))]);
ylabel('Position [m]'); %xlabel('Time [s]');
legend('x','y','z','$$x_d$$','$$y_d$$','$$z_d$$');

subplot(2,1,2);
plot(t,[phi theta psi psid(1:length(psi)) phiL thetaL]);
legend('$$\phi$$','$$\theta$$','$$\psi$$','$$\psi_d$$','$$\phi_L$$','$$\theta_L$$');
ylabel('Aircraft/Load Orientation[rad]');

xlabel('Time [s]');
fig = gcf;
title(fig.Children(end), 'Robot State');


%% Plot Robot Lienar and Angular Velocities
figure;

subplot(2,1,1);
plot(t,[xdot ydot zdot xdot_d ydot_d zdot_d]);
legend('$$\dot{x}$$','$$\dot{y}$$','$$\dot{z}$$','$$\dot{x}_d$$','$$\dot{y}_d$$','$$\dot{z}_d$$');
ylabel('Linear Velocity [m/s]');

subplot(2,1,2);
plot(t,[phidot thetadot psidot phiLdot thetaLdot psidot_d]);
ylabel('Angular Velocity [rad/s]');
legend('$$\dot{\phi}$$','$$\dot{\theta}$$','$$\dot{\psi}$$','$$\dot{\phi}_L$$','$$\dot{\theta}_L$$','$$\dot{\psi}_d$$');

xlabel('Time [s]');
fig = gcf;
title(fig.Children(end), 'Robot Twist');


%% Plot Control Input
figure;
subplot(2,1,1);
plot(t,u(:,1),t,ueq(:,1),t,usw(:,1));
ylabel('Thrust Force $$U_1$$ [N]');
legend('$$U_1$$(actual)', '$$U_{1eq}$$', '$$U_{1sw}$$');
subplot(2,1,2);
plot(t,u(:,2:end));
ylabel('Input Torque [N$$\cdot$$m]');
legend('$$U_2$$','$$U_3$$','$$U_4$$');

xlabel('Time [s]');
fig = gcf;
title(fig.Children(end), 'Control Input');

% Plot sliding variables
figure;
plot(t,s);
ylabel('Sliding variables');
legend('$$s_1$$','$$s_2$$', '$$s_3$$', '$$s_4$$');

xlabel('Time [s]');
fig = gcf;
title(fig.Children(end), 'Sliding Variables');

end