function plot_quadrotor_rpy_pt(t,rpy, rpy_d, rpy_dot, rpy_dot_d)

phi = rpy(:,1);
theta = rpy(:,2);
psi = rpy(:,3);

phi_d = rpy_d(:,1);
theta_d = rpy_d(:,2);
psi_d = rpy_d(:,3);

phidot = rpy_dot(:,1);
thetadot = rpy_dot(:,2);
psidot = rpy_dot(:,3);

phidot_d = rpy_dot_d(:,1);
thetadot_d = rpy_dot_d(:,2);
psidot_d = rpy_dot_d(:,3);


%% Plot Robot Orientation and Euler Angles Rate
ylim_phitheta = calc_ylim([rpy(:,1:2);rpy_d(:,1:2)],.1);
ylim_psi = calc_ylim([psi;psi_d],.1);
ylim_rpy_dot = calc_ylim([rpy_dot;rpy_dot_d],.1);

figure;
set(gcf, 'OuterPosition', [300, 150, 940, 770]);
subplot(3,2,1);
plot(t,phi_d,'--', t, phi);
ylabel('$\phi$ [rad]');
legend('Desejado','Realizado');
th = title('Orientação da Aeronave', 'Interpreter','tex','FontSize',12,'FontAngle','Italic','FontWeight','normal');
set(th,'Unit','normalized','Position',[.5, 1.1, 0]);
ylim(ylim_phitheta);
xlim([t(1) t(end)]);

subplot(3,2,2);
plot(t,phidot_d,'--', t, phidot);
ylabel('$\dot{\phi}$ [rad/s]');
legend('Desejado','Realizado');
th = title('Velocidade Angular da Aeronave', 'Interpreter','tex','FontSize',12,'FontAngle','Italic','FontWeight','normal');
set(th,'Unit','normalized','Position',[.5, 1.1, 0]);
ylim(ylim_rpy_dot);
xlim([t(1) t(end)]);


subplot(3,2,3);
plot(t,theta_d,'--', t, theta);
ylabel('$$\theta$$ [rad]');
legend('Desejado','Realizado');
ylim(ylim_phitheta);
xlim([t(1) t(end)]);

subplot(3,2,4);
plot(t,thetadot_d,'--', t, thetadot);
ylabel('$$\dot{\theta}$$ [rad/s]');
legend('Desejado','Realizado');
ylim(ylim_rpy_dot);
xlim([t(1) t(end)]);

subplot(3,2,5);
plot(t,psi_d,'--', t, psi);
ylabel('$$\psi$$ [rad]');
legend('Desejado','Realizado');
xlabel('Tempo [s]','Interpreter','tex','FontSize',12);
ylim(ylim_psi);
xlim([t(1) t(end)]);

subplot(3,2,6);
plot(t,psidot_d,'--', t, psidot);
ylabel('$$\dot{\psi}$$ [rad/s]');
legend('Desejado','Realizado');
xlabel('Tempo [s]','Interpreter','tex','FontSize',12);
ylim(ylim_rpy_dot);
xlim([t(1) t(end)]);

end

