function plot_horizontal_state(t, xy, phitheta, phithetaL)

%% State remapping
x = xy(:,1);
y = xy(:,2);

phi = phitheta(:,1);
theta = phitheta(:,2);

phiL = phithetaL(:,1);
thetaL = phithetaL(:,2);

%% Plot Robot Orientation and Euler Angles Rate
ylim_pos = calc_ylim(xy,.1);
ylim_ang = calc_ylim([phitheta, phithetaL],.1);

figure;
set(gcf, 'OuterPosition', [300, 150, 1200, 500]);

subplot(2,3,1);
plot(t, zeros(size(t)), '--', t,x);
ylabel('$x$ [m]');
legend('Desejado','Realizado');
th = title('Posição da Aeronave', 'Interpreter','tex','FontSize',12,'FontAngle','Italic','FontWeight','normal');
set(th,'Unit','normalized','Position',[.5, 1.1, 0]);
ylim(ylim_pos);
xlim([t(1) t(end)]);

subplot(2,3,4);
plot(t, zeros(size(t)), '--', t,y);
ylabel('$y$ [m]');
legend('Desejado','Realizado');
ylim(ylim_pos);
xlim([t(1) t(end)]);
xlabel('Tempo [s]','Interpreter','tex','FontSize',12);


subplot(2,3,2);
plot(t, zeros(size(t)), '--', t,phi);
ylabel('$\phi$ [rad]');
legend('Desejado','Realizado');
th = title('Orientação da Aeronave', 'Interpreter','tex','FontSize',12,'FontAngle','Italic','FontWeight','normal');
set(th,'Unit','normalized','Position',[.5, 1.1, 0]);
ylim(ylim_ang);
xlim([t(1) t(end)]);

subplot(2,3,5);
plot(t, zeros(size(t)), '--', t,theta);
ylabel('$\theta$ [rad]');
legend('Desejado','Realizado');
ylim(ylim_ang);
xlim([t(1) t(end)]);
xlabel('Tempo [s]','Interpreter','tex','FontSize',12);

subplot(2,3,3);
plot(t, zeros(size(t)), '--', t,phiL);
ylabel('$\phi_L$ [rad]');
legend('Desejado','Realizado');
th = title('Orientação do Cabo', 'Interpreter','tex','FontSize',12,'FontAngle','Italic','FontWeight','normal');
set(th,'Unit','normalized','Position',[.5, 1.1, 0]);
ylim(ylim_ang);
xlim([t(1) t(end)]);

subplot(2,3,6);
plot(t, zeros(size(t)), '--', t,thetaL);
ylabel('$\theta_L$ [rad]');
legend('Desejado','Realizado');
ylim(ylim_ang);
xlim([t(1) t(end)]);
xlabel('Tempo [s]','Interpreter','tex','FontSize',12);

end

