function plot_cable_angle_pt(t,phithetaL,phithetaL_d,phithetaL_dot,phithetaL_dot_d,stop_time)

phiL = phithetaL(:,1);
thetaL = phithetaL(:,2);
% alphaL = acos(cos(phiL).*cos(thetaL));

phiLdot = phithetaL_dot(:,1);
thetaLdot = phithetaL_dot(:,2);

phiL_d = phithetaL_d(:,1);
thetaL_d = phithetaL_d(:,2);
% alphaL_d = asin(sqrt(sin(thetaL).^2 + sin(phiL).^2.*cos(thetaL).^2));

phiLdot_d = phithetaL_dot_d(:,1);
thetaLdot_d = phithetaL_dot_d(:,2);

%% Plot Robot Orientation and Euler Angles Rate
ylim_phithetaL = calc_ylim([phithetaL phithetaL_d],.2);
ylim_phithetaLdot = calc_ylim([phithetaL_dot phithetaL_dot_d],.2);
% ylim_alphaL = calc_ylim([psi;psi_d],.1);

figure;
subplot(2,2,1);
plot(t,phiL_d,'--', t, phiL);
line([stop_time, stop_time], ylim_phithetaL,'Color','k','LineWidth',.8,'LineStyle',':');
ylabel('$\phi_L$ [rad]');
legend('Desejado','Realizado');
th = title('Orientação do Cabo', 'Interpreter','tex','FontSize',12,'FontAngle','Italic','FontWeight','normal');
set(th,'Unit','normalized','Position',[.5, 1.1, 0]);
ylim(ylim_phithetaL);
xlim([t(1) t(end)]);

subplot(2,2,2);
plot(t,phiLdot_d,'--', t, phiLdot);
line([stop_time, stop_time], ylim_phithetaLdot,'Color','k','LineWidth',.8,'LineStyle',':');
ylabel('$\dot{\phi}_L$ [rad/s]');
legend('Desejado','Realizado');
th = title('Taxa de Rotação do Cabo', 'Interpreter','tex','FontSize',12,'FontAngle','Italic','FontWeight','normal');
set(th,'Unit','normalized','Position',[.5, 1.1, 0]);
ylim(ylim_phithetaLdot);
xlim([t(1) t(end)]);


subplot(2,2,3);
plot(t,thetaL_d,'--', t, thetaL);
line([stop_time, stop_time], ylim_phithetaL,'Color','k','LineWidth',.8,'LineStyle',':');
ylabel('$$\theta_L$$ [rad]');
xlabel('Tempo [s]','Interpreter','tex','FontSize',12);
legend('Desejado','Realizado');
ylim(ylim_phithetaL);
xlim([t(1) t(end)]);

subplot(2,2,4);
plot(t,thetaLdot_d,'--', t, thetaLdot);
line([stop_time, stop_time], ylim_phithetaLdot,'Color','k','LineWidth',.8,'LineStyle',':');
ylabel('$$\dot{\theta}_L$$ [rad/s]');
legend('Desejado','Realizado');
xlabel('Tempo [s]','Interpreter','tex','FontSize',12);
ylim(ylim_phithetaLdot);
xlim([t(1) t(end)]);

% set(gcf, 'OuterPosition', [300, 150, 940, 520]);
set(gcf, 'OuterPosition', [300, 150, 650, 520]);
% figure;
% plot(t,alphaL);

end

