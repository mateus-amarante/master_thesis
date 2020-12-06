function plot_quadrotor3d_slung_flat_state(t,q,physics_p, control_p, traj_p, dict)

% State remapping
x = q;
q = x(:,1:end/2);
qdot = x(:,end/2+1:end);

% Compute desired trajectory
qd = traj_p.sample_fun(t);

u = zeros(length(t), control_p.n_inputs);
u_d = qd(:, 25:end);
s = u;
ueq = u;
usw = u;

% TODO: define u for all timespecs at once
for i=1:length(t)
    [u(i,:), s(i,:), ueq(i,:), usw(i,:)] = control_p.control_fun(x(i,:),qd(i,:),physics_p,control_p);
end

% Robot Position & orientation
xyz = q(:,1:3);
rpy = q(:,4:6);
phithetaL = q(:,7:8);

% Robot Velocity
xyz_dot = qdot(:,1:3);
rpy_dot = qdot(:,4:6);
phithetaL_dot = qdot(:,7:8);

% Desired position
xyz_d = qd(:,1:3);
rpy_d = qd(:,4:6);
phithetaL_d = qd(:,7:8);

% Desired velocity
xyz_dot_d = qd(:,9:11);
rpy_dot_d = qd(:,12:14);
phithetaL_dot_d = qd(:,15:16);

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

phid = qd(:,4);
thetad = qd(:,5);
psid = qd(:,6);

phiLd = qd(:,7);
thetaLd = qd(:,8);


xdot_d = qd(:,9);
ydot_d = qd(:,10);
zdot_d = qd(:,11);

phidot_d = qd(:,12);
thetadot_d = qd(:,13);
psidot_d = qd(:,14);

phiLdot_d = qd(:,15);
thetaLdot_d = qd(:,16);

%% Plot Robot State
plot_quadrotor_posvel(t,xyz, xyz_d, xyz_dot, xyz_dot_d, dict);
plot_quadrotor_rpy(t,rpy, rpy_d, rpy_dot, rpy_dot_d, dict);
plot_sliding_variables(t,s, dict);
plot_cable_angle(t,phithetaL,phithetaL_d,phithetaL_dot,phithetaL_dot_d,traj_p.stop_time, dict);
plot_control_input_flat(t,u,u_d, dict);

% 
% figure;
% subplot(3,2,1);
% plot(t,[x y z xd(1:length(x)) yd(1:length(y)) zd(1:length(z))]);
% ylabel('Posição [m]'); %xlabel('Time [s]');
% legend('x','y','z','$$x_d$$','$$y_d$$','$$z_d$$');
% 
% subplot(3,2,2);
% plot(t,[phi theta psi phid thetad psid]);
% legend('$$\phi$$','$$\theta$$','$$\psi$$','$$\phi_d$$','$$\theta_d$$','$$\psi_d$$');
% ylabel('Orientação da Aeronave [rad]');
% 
% subplot(3,2,4);
% plot(t,[phiL, thetaL]);
% legend('$$\phi$$','$$\theta$$','$$\psi$$','$$\phi_d$$','$$\theta_d$$','$$\psi_d$$');
% ylabel('Orientação da Aeronave [rad]');
% 
% xlabel('Time [s]');
% fig = gcf;
% title(fig.Children(end), 'Robot State');

% % % Plot Load State
% % xL = x - physics_p.L*sin(thetaL);
% % yL = y + physics_p.L*sin(phiL).*cos(thetaL);
% % zL = z - physics_p.L*cos(phiL).*cos(thetaL);
% % 
% % xL_d = xd - physics_p.L*sin(thetaLd);
% % yL_d = yd + physics_p.L*sin(phiLd).*cos(thetaLd);
% % zL_d = zd - physics_p.L*cos(phiLd).*cos(thetaLd);
% % 
% % figure;
% % subplot(2,1,1);
% % plot(t,[xL yL zL xL_d yL_d zL_d]);
% % ylabel('Position [m]'); %xlabel('Time [s]');
% % legend('$$x_L$$','$$y_L$$','$$z_L$$','$${x_L}_d$$','$${y_L}_d$$','$${z_L}_d$$');
% % 
% % subplot(2,1,2);
% % plot(t,[phiL thetaL phiLd thetaLd]);
% % legend('$$\phi_L$$','$$\theta_L$$','$${\phi_L}_d$$','$${\theta_L}_d$$');
% % ylabel('Load Orientation[rad]');
% % 
% % xlabel('Time [s]');
% % fig = gcf;
% % title(fig.Children(end), 'Load State');
% % 
% % 
% % % Plot Robot Linear and Angular Velocities
% % figure;
% % 
% % subplot(2,1,1);
% % plot(t,[xdot ydot zdot xdot_d ydot_d zdot_d]);
% % legend('$$\dot{x}$$','$$\dot{y}$$','$$\dot{z}$$','$$\dot{x}_d$$','$$\dot{y}_d$$','$$\dot{z}_d$$');
% % ylabel('Linear Velocity [m/s]');
% % 
% % subplot(2,1,2);
% % plot(t,[phidot thetadot psidot phidot_d thetadot_d psidot_d]);
% % ylabel('Angular Velocity [rad/s]');
% % legend('$$\dot{\phi}$$','$$\dot{\theta}$$','$$\dot{\psi}$$','$$\dot{\phi}_d$$','$$\dot{\theta}_d$$','$$\dot{\psi}_d$$');
% % 
% % xlabel('Time [s]');
% % fig = gcf;
% % title(fig.Children(end), 'Robot Twist');
% % 
% % % Plot Load Linear and Angular Velocities
% % xLdot = xdot - physics_p.L*cos(thetaL).*thetaLdot;
% % yLdot = ydot + physics_p.L*(cos(phiL).*cos(thetaL).*phiLdot - sin(phiL).*sin(thetaL).*thetaLdot);
% % zLdot = zdot + physics_p.L*(sin(phiL).*cos(thetaL).*phiLdot + cos(phiL).*sin(thetaL).*thetaLdot);
% % 
% % xLdot_d = xdot_d - physics_p.L*cos(thetaLd).*thetaLdot_d;
% % yLdot_d = ydot_d + physics_p.L*(cos(phiLd).*cos(thetaLd).*phiLdot_d - sin(phiLd).*sin(thetaLd).*thetaLdot_d);
% % zLdot_d = zdot_d + physics_p.L*(sin(phiLd).*cos(thetaLd).*phiLdot_d + cos(phiLd).*sin(thetaLd).*thetaLdot_d);
% % 
% % figure;
% % 
% % subplot(2,1,1);
% % plot(t,[xLdot yLdot zLdot xLdot_d yLdot_d zLdot_d]);
% % legend('$$\dot{x}_L$$','$$\dot{y}_L$$','$$\dot{z}_L$$','$$\dot{x}_{L_d}$$','$$\dot{y}_{L_d}$$','$$\dot{z}_{L_d}$$');
% % ylabel('Linear Velocity [m/s]');
% % 
% % subplot(2,1,2);
% % plot(t,[phiLdot thetaLdot phiLdot_d thetaLdot_d]);
% % ylabel('Angular Velocity [rad/s]');
% % legend('$$\dot{\phi}_L$$','$$\dot{\theta}_L$$','$$\dot{\phi}_{L_d}$$','$$\dot{\theta}_{L_d}$$');
% % 
% % xlabel('Time [s]');
% % fig = gcf;
% % title(fig.Children(end), 'Load Twist');
% % 
% % 
% % % Plot Control Input
% % figure;
% % subplot(2,1,1);
% % plot(t,u(:,1),t,qd(:, 25),t,ueq(:,1),t,usw(:,1));
% % ylabel('Thrust Force $$U_1$$ [N]');
% % legend('$$U_1$$(actual)','$$U_1$$(expected)', '$$U_{1eq}$$', '$$U_{1sw}$$');
% % 
% % subplot(2,1,2);
% % plot(t,u(:,2:end),t,qd(:, 26:end));
% % ylabel('Input Torque [N$$\cdot$$m]');
% % legend('$$U_2$$','$$U_3$$','$$U_4$$','$$U_2$$(expected)','$$U_3$$(expected)','$$U_4$$(expected)');
% % 
% % xlabel('Time [s]');
% % fig = gcf;
% % title(fig.Children(end), 'Control Input');
% % 
% % Plot sliding variables
% % figure;
% % plot(t,s);
% % ylabel('Sliding variables');
% % legend('$$s_1$$','$$s_2$$', '$$s_3$$', '$$s_4$$');
% % 
% % xlabel('Time [s]');
% % fig = gcf;
% % title(fig.Children(end), 'Sliding Variables');

end