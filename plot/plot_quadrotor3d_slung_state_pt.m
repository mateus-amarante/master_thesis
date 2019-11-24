function plot_quadrotor3d_slung_state_pt(t,q,physics_p, control_p, traj_p)

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
phithetaL_d = zeros(length(t),2);

% Desired velocity
xyz_dot_d = qd(:,9:11);
rpy_dot_d = qd(:,12:14);
phithetaL_dot_d = zeros(length(t),2);

%% Plot Robot State
plot_quadrotor_posvel_pt(t,xyz, xyz_d, xyz_dot, xyz_dot_d);
plot_quadrotor_rpy_pt(t,rpy, rpy_d, rpy_dot, rpy_dot_d);
plot_sliding_variables_pt(t,s);
plot_cable_angle_pt(t,phithetaL,phithetaL_d,phithetaL_dot,phithetaL_dot_d,traj_p.stop_time);
plot_control_input_pt(t,u);
plot_horizontal_state(t,xyz,rpy,phithetaL);
% plot_phase_portrait(xyz(:,3), xyz_dot(:,3), '$z$', '$\dot{z}$', control_p.lambda_zpsi(1));
% plot_phase_portrait(rpy(:,3), rpy_dot(:,3), '$\psi$', '$\dot{\psi}$', control_p.lambda_zpsi(2));
% plot_phase_portrait(rpy(:,1), rpy_dot(:,1), '$\phi$', '$\dot{\phi}$',0);
% plot_phase_portrait(rpy(:,2), rpy_dot(:,2), '$\theta$', '$\dot{\theta}$',0);
% plot_phase_portrait(xyz(:,1), xyz_dot(:,1), '$x$', '$\dot{x}$',0);
% plot_phase_portrait(xyz(:,2), xyz_dot(:,2), '$y$', '$\dot{y}$',0);

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