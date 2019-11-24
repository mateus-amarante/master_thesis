function plot_sliding_variables_pt(t, s, sdot, lambda)


%% Plot Robot Orientation and Euler Angles Rate
ylim_s = calc_ylim(s,.1);

figure;
% set(gcf, 'OuterPosition', [300, 150, 940, 770]);
% subplot(3,2,1);
plot(t,phi_d,'--', t, phi);
ylabel('$\phi$ [m]');
legend('Desejado','Realizado');
th = title('Orientação da Aeronave', 'Interpreter','tex','FontSize',12,'FontAngle','Italic','FontWeight','normal');
set(th,'Unit','normalized','Position',[.5, 1.1, 0]);
ylim(ylim_phitheta);

subplot(3,2,2);
plot(t,phidot_d,'--', t, phidot);
ylabel('$\dot{\phi}$ [m/s]');
legend('Desejado','Realizado');
th = title('Velocidade Angular da Aeronave', 'Interpreter','tex','FontSize',12,'FontAngle','Italic','FontWeight','normal');
set(th,'Unit','normalized','Position',[.5, 1.1, 0]);
ylim(ylim_rpy_dot);

subplot(3,2,3);
plot(t,theta_d,'--', t, theta);
ylabel('$$\theta$$ [m]');
legend('Desejado','Realizado');
ylim(ylim_phitheta);


subplot(3,2,4);
plot(t,thetadot_d,'--', t, thetadot);
ylabel('$$\dot{\theta}$$ [m/s]');
legend('Desejado','Realizado');
ylim(ylim_rpy_dot);

subplot(3,2,5);
plot(t,psi_d,'--', t, psi);
ylabel('$$\psi$$ [m]');
legend('Desejado','Realizado');
xlabel('Tempo [s]','Interpreter','tex','FontSize',12);
ylim(ylim_psi);

subplot(3,2,6);
plot(t,psidot_d,'--', t, psidot);
ylabel('$$\dot{\psi}$$ [m/s]');
legend('Desejado','Realizado');
xlabel('Tempo [s]','Interpreter','tex','FontSize',12);
ylim(ylim_rpy_dot);

end

