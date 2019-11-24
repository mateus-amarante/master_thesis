function plot_control_input_pt(t, u)

figure;
set(gcf, 'OuterPosition', [100, 50, 500, 900]);
subplot(4,1,1);
plot(t, u(:,1));
ylabel('$u_1$ [N]');
% legend('Esperado','Realizado');
th = title('Esfor�os de Controle', 'Interpreter','tex','FontSize',12,'FontAngle','Italic','FontWeight','normal');
set(th,'Unit','normalized','Position',[.5, 1.1, 0]);
ylim(calc_ylim([u(:,1)],.1));
xlim([t(1) t(end)]);

subplot(4,1,2);
plot(t, u(:,2));
ylabel('$u_2$ [N.m]');
% legend('Esperado','Realizado');
ylim(calc_ylim([u(:,2)],.1));
xlim([t(1) t(end)]);

subplot(4,1,3);
plot(t, u(:,3));
ylabel('$u_3$ [N.m]');
% legend('Esperado','Realizado');
ylim(calc_ylim([u(:,3)],.1));
xlim([t(1) t(end)]);

subplot(4,1,4);
plot(t, u(:,4));
ylabel('$u_4$ [N.m]');
% legend('Esperado','Realizado');
ylim(calc_ylim([u(:,4)],.1));
xlim([t(1) t(end)]);
xlabel('Tempo [s]','Interpreter','tex','FontSize',12);

end

