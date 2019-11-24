function plot_control_input_flat_pt(t, u, u_d)

figure;
set(gcf, 'OuterPosition', [100, 50, 500, 900]);
subplot(4,1,1);
plot(t,u_d(:,1),'--', t, u(:,1));
ylabel('$u_1$ [N]');
legend('Esperado','Realizado');
th = title('Esforços de Controle', 'Interpreter','tex','FontSize',12,'FontAngle','Italic','FontWeight','normal');
set(th,'Unit','normalized','Position',[.5, 1.1, 0]);
% ylim(calc_ylim([u_d(:,1);u(:,1)],.1));
xlim([t(1) t(end)]);

subplot(4,1,2);
plot(t,u_d(:,2),'--', t, u(:,2));
ylabel('$u_2$ [N.m]');
legend('Esperado','Realizado');
% ylim(calc_ylim([u_d(:,1);u(:,1)],.1));
xlim([t(1) t(end)]);

subplot(4,1,3);
plot(t,u_d(:,3),'--', t, u(:,3));
ylabel('$u_3$ [N.m]');
legend('Esperado','Realizado');
% ylim(calc_ylim([u_d(:,1);u(:,1)],.1));
xlim([t(1) t(end)]);

subplot(4,1,4);
plot(t,u_d(:,4),'--', t, u(:,4));
ylabel('$u_4$ [N.m]');
legend('Esperado','Realizado');
% ylim(calc_ylim([u_d(:,1);u(:,1)],.1));
xlim([t(1) t(end)]);
xlabel('Tempo [s]','Interpreter','tex','FontSize',12);

end

