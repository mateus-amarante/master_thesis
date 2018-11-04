function plot_quadrotor3d_slung_state(t,q,qdot,qd,u,physics_p, control_p)

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
psid = qd(:,4);

xdot_d = qd(:,5);
ydot_d = qd(:,6);
zdot_d = qd(:,7);
psidot_d = qd(:,8);


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
plot(t,u(:,1));
ylabel('Thrust Force $$U_1$$ [N]');

subplot(2,1,2);
plot(t,u(:,2:end));
ylabel('Input Torque [N$$\cdot$$m]');
legend('$$U_2$$','$$U_3$$','$$U_4$$');

xlabel('Time [s]');
fig = gcf;
title(fig.Children(end), 'Control Input');

%% Plot sliding variables
for i=1:length(t)
    % FIXME: state variables are temporarily transposed for nested3d_control
    [~, ss] = control_p.control_fun(q(i,:)',qdot(i,:)',qd(i,:)',physics_p,control_p);
    s(:,i) = ss;
end

sdot = diff(s')/(t(2)-t(1));
% 
figure;
% plot(s(3,1:end-1),sdot(:,3));
plot(t,s.^2);

end