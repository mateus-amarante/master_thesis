startup;

[physics_p, control_p, traj_p, sim_p, plot_p] = flat_smc3d_slung_config();
[physics_p_s, control_p_s, traj_p_s, sim_p_s, plot_p_s] = shaped_flat_smc3d_slung_config();

qd = traj_p.sample_fun(sim_p.t);
qd_s = traj_p_s.sample_fun(sim_p_s.t);
rL = traj_p.rL_fun(sim_p.t);
rL_s = traj_p_s.rL_fun(sim_p_s.t);

xyz = qd(:,1:3);
rpy = qd(:,4:6);
phithetaL = qd(:,7:8);
rpydot = qd(:,12:14);

xyz_s = qd_s(:,1:3);
rpy_s = qd_s(:,4:6);
phithetaL_s = qd_s(:,7:8);
rpydot_s = qd_s(:,12:14);

%% Plot path
fig = figure;
fig.Renderer='Painters';
xyz_load = zeros(size(xyz));
xyz_load_s = xyz_load;
t = sim_p.t;

for i=1:length(t)
    xyz_load(i,1:3) = xyz(i,:) + (eul2rotm([phithetaL(i,:), 0],'XYZ')*[0; 0; -physics_p.l])';
    xyz_load_s(i,1:3) = xyz_s(i,:) + (eul2rotm([phithetaL_s(i,:), 0],'XYZ')*[0; 0; -physics_p.l])';
end
plot_quadrotor3d(xyz(1,:),rpy(1,:),xyz_load(1,:),physics_p.r,physics_p.rotor_r,physics_p.load_radius);
h1 = line(xyz_load(:,1),xyz_load(:,2),xyz_load(:,3),'Color','r');
h2 = line(xyz(:,1),xyz(:,2),xyz(:,3),'Color','g');
h3 = line(xyz_s(:,1),xyz_s(:,2),xyz_s(:,3),'Color','b');
legend([h1,h2,h3],dict.load_trajectory_leg, dict.quad_trajectory_without_input_shaping_leg, dict.quad_trajectory_with_input_shaping_leg);
grid on;
xlabel('$x$ [m]');ylabel('$y$ [m]');zlabel('$z$ [m]');
% line(xyz_load_s(:,1),xyz_load_s(:,2),xyz_load_s(:,3),'Color','r');



%% Convert angles to degree
rpy = rad2deg(rpy);
rpy_s = rad2deg(rpy_s);
phithetaL = rad2deg(phithetaL);
phithetaL_s = rad2deg(phithetaL_s);
rpydot = rad2deg(rpydot);
rpydot_s = rad2deg(rpydot_s);

dict = plot_dictionary('pt');

figure;
set(gcf, 'OuterPosition', [300, 150, 940, 770]);
subplot(3,2,1);
plot(sim_p.t, rL(:,1), sim_p_s.t, rL_s(:,1));
legend(dict.without_input_shaping_leg, dict.with_input_shaping_leg);
ylabel('$x_L$ [m]');
xlim([0, sim_p.t(end)]);
th = title(dict.load_position_title, 'Interpreter','tex','FontSize',12,'FontAngle','Italic','FontWeight','normal');
set(th,'Unit','normalized','Position',[.5, 1.1, 0]);
ylim(calc_ylim(rL(:,1),.1));

subplot(3,2,2);
plot(sim_p.t, qd(:,1), sim_p_s.t, qd_s(:,1));
legend(dict.without_input_shaping_leg, dict.with_input_shaping_leg);
ylabel('$x$ [m]');
xlim([0, sim_p.t(end)]);
th = title(dict.quad_position_title, 'Interpreter','tex','FontSize',12,'FontAngle','Italic','FontWeight','normal');
set(th,'Unit','normalized','Position',[.5, 1.1, 0]);
ylim(calc_ylim(qd(:,1),.1));


subplot(3,2,3);
plot(sim_p.t, rL(:,2), sim_p_s.t, rL_s(:,2));
legend(dict.without_input_shaping_leg, dict.with_input_shaping_leg);
ylabel('$y_L$ [m]');
xlim([0, sim_p.t(end)]);
ylim(calc_ylim(rL(:,2),.1));

subplot(3,2,4);
plot(sim_p.t, qd(:,2), sim_p_s.t, qd_s(:,2));
legend(dict.without_input_shaping_leg, dict.with_input_shaping_leg);
ylabel('$y$ [m]');
xlim([0, sim_p.t(end)]);
ylim(calc_ylim(qd(:,2),.1));

subplot(3,2,5);
plot(sim_p.t, rL(:,3), sim_p_s.t, rL_s(:,3));
legend(dict.without_input_shaping_leg, dict.with_input_shaping_leg);
ylabel('$z_L$ [m]');
xlabel(dict.time_label,'Interpreter','tex','FontSize',12);
xlim([0, sim_p.t(end)]);
ylim(calc_ylim([rL(:,3), qd(:,3)],.1));

subplot(3,2,6);
plot(sim_p.t, qd(:,3), sim_p_s.t, qd_s(:,3));
legend(dict.without_input_shaping_leg, dict.with_input_shaping_leg);
ylabel('$z$ [m]');
xlabel(dict.time_label,'Interpreter','tex','FontSize',12);
xlim([0, sim_p.t(end)]);
ylim(calc_ylim([rL(:,3), qd(:,3)],.1));

%% 
figure;
subplot(3,1,1);
plot(sim_p.t, rpy(:,1), sim_p_s.t, rpy_s(:,1));
legend(dict.without_input_shaping_leg, dict.with_input_shaping_leg);
ylabel('$$\phi [{}^{\circ}]$$');
th = title(dict.quad_orientation_title, 'Interpreter','tex','FontSize',12,'FontAngle','Italic','FontWeight','normal');
set(th,'Unit','normalized','Position',[.5, 1.1, 0]);
xlim([0, sim_p.t(end)]);
ylim(calc_ylim([rpy(:,1), rpy_s(:,1)],.1));

subplot(3,1,2);
plot(sim_p.t, rpy(:,2), sim_p_s.t, rpy_s(:,2));
legend(dict.without_input_shaping_leg, dict.with_input_shaping_leg);
ylabel('$$\theta [{}^{\circ}]$$');
xlim([0, sim_p.t(end)]);
ylim(calc_ylim([rpy(:,2), rpy_s(:,2)],.1));

subplot(3,1,3);
plot(sim_p.t, rpy(:,3), sim_p_s.t, rpy_s(:,3));
legend(dict.without_input_shaping_leg, dict.with_input_shaping_leg);
ylabel('$$\psi [{}^{\circ}]$$');
xlabel(dict.time_label,'Interpreter','tex','FontSize',12);
xlim([0, sim_p.t(end)]);
ylim(calc_ylim([rpy(:,3), rpy_s(:,3)],.1));


figure;
subplot(2,1,1);
plot(sim_p.t, phithetaL(:,1), sim_p_s.t, phithetaL_s(:,1));
legend(dict.without_input_shaping_leg, dict.with_input_shaping_leg);
ylabel('$$\phi_L [{}^{\circ}]$$');
th = title(dict.cable_orientation_title, 'Interpreter','tex','FontSize',12,'FontAngle','Italic','FontWeight','normal');
set(th,'Unit','normalized','Position',[.5, 1.1, 0]);
xlim([0, sim_p.t(end)]);
ylim(calc_ylim([phithetaL(:,1), phithetaL_s(:,1)],.1));

subplot(2,1,2);
plot(sim_p.t, phithetaL(:,2), sim_p_s.t, phithetaL_s(:,2));
legend(dict.without_input_shaping_leg, dict.with_input_shaping_leg);
ylabel('$$\theta_L [{}^{\circ}]$$');
xlabel(dict.time_label,'Interpreter','tex','FontSize',12);
xlim([0, sim_p.t(end)]);
ylim(calc_ylim([phithetaL(:,2), phithetaL_s(:,2)],.1));


% plot(sim_p.t, qd(:,2), sim_p_s.t, qd_s(:,2));
% plot(sim_p.t, qd(:,3), sim_p_s.t, qd_s(:,3));
% plot(sim_p.t, qd(:,4), sim_p_s.t, qd_s(:,4));
% plot(sim_p.t, qd(:,5), sim_p_s.t, qd_s(:,5));
% plot(sim_p.t, qd(:,6), sim_p_s.t, qd_s(:,6));

figure;
subplot(3,1,1);
plot(sim_p.t, rpydot(:,1), sim_p_s.t, rpydot_s(:,1));
legend(dict.without_input_shaping_leg, dict.with_input_shaping_leg);
ylabel('$$\dot{\phi} [{}^{\circ}/s]$$');
th = title(dict.quad_angular_velocity_euler_title, 'Interpreter','tex','FontSize',12,'FontAngle','Italic','FontWeight','normal');
set(th,'Unit','normalized','Position',[.5, 1.1, 0]);
xlim([0, sim_p.t(end)]);
ylim(calc_ylim([rpydot(:,1), rpydot_s(:,1)],.1));

subplot(3,1,2);
plot(sim_p.t, rpydot(:,2), sim_p_s.t, rpydot_s(:,2));
legend(dict.without_input_shaping_leg, dict.with_input_shaping_leg);
ylabel('$$\dot{\theta} [{}^{\circ}/s]$$');
xlim([0, sim_p.t(end)]);
ylim(calc_ylim([rpydot(:,2), rpydot_s(:,2)],.1));

subplot(3,1,3);
plot(sim_p.t, rpydot(:,3), sim_p_s.t, rpydot_s(:,3));
legend(dict.without_input_shaping_leg, dict.with_input_shaping_leg);
ylabel('$$\dot{\psi} [{}^{\circ}/s]$$');
xlabel(dict.time_label,'Interpreter','tex','FontSize',12);
xlim([0, sim_p.t(end)]);
ylim(calc_ylim([rpydot(:,3), rpydot_s(:,3)],.1));

%% System input
figure;
% set(gcf, 'OuterPosition', [300, 150, 940, 770]);
subplot(4,1,1);
plot(t,qd(:,25), t, qd_s(:,25));
ylabel('$u_1$ [N]');
legend(dict.without_input_shaping_leg, dict.with_input_shaping_leg);
th = title(dict.control_input_title, 'Interpreter','tex','FontSize',12,'FontAngle','Italic','FontWeight','normal');
set(th,'Unit','normalized','Position',[.5, 1.1, 0]);
% ylim(calc_ylim([u_d(:,1);u(:,1)],.1));
xlim([t(1) t(end)]);
ylim(calc_ylim([qd(:,25), qd_s(:,25)],.1));

subplot(4,1,2);
plot(t,qd(:,26), t, qd_s(:,26));
ylabel('$u_2$ [N.m]');
legend(dict.without_input_shaping_leg, dict.with_input_shaping_leg);
% ylim(calc_ylim([u_d(:,1);u(:,1)],.1));
xlim([t(1) t(end)]);
ylim(calc_ylim([qd(:,26), qd_s(:,26)],.1));

subplot(4,1,3);
plot(t,qd(:,27), t, qd_s(:,27));
ylabel('$u_3$ [N.m]');
legend(dict.without_input_shaping_leg, dict.with_input_shaping_leg);
% ylim(calc_ylim([u_d(:,1);u(:,1)],.1));
xlim([t(1) t(end)]);
ylim(calc_ylim([qd(:,27), qd_s(:,27)],.1));

subplot(4,1,4);
plot(t,qd(:,28), t, qd_s(:,28));
ylabel('$u_4$ [N.m]');
legend(dict.without_input_shaping_leg, dict.with_input_shaping_leg);
% ylim(calc_ylim([u_d(:,1);u(:,1)],.1));
xlim([t(1) t(end)]);
xlabel(dict.time_label,'Interpreter','tex','FontSize',12);
ylim(calc_ylim([qd(:,28), qd_s(:,28)],.1));





% plot_p.plot_animation(sim_p.t, qd, physics_p, traj_p,sim_p, true);
% plot_p_s.plot_animation(sim_p.t, qd_s, physics_p_s, traj_p_s);