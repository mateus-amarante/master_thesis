function plot_control_input_flat(t, u, u_d, dict)

plot_control_input(t,u,dict);
subplot(4,1,1);
plot(t,u_d(:,1),'--', t, u(:,1));
legend(dict.desired_leg, dict.actual_leg);
ylim(calc_ylim([u_d(:,1);u(:,1)],.1));
ylabel('$u_1$ [N]');

subplot(4,1,2);
plot(t,u_d(:,2),'--', t, u(:,2));
legend(dict.desired_leg, dict.actual_leg);
ylim(calc_ylim([u_d(:,2);u(:,3)],.1));
ylabel('$u_2$ [N.m]');


subplot(4,1,3);
plot(t,u_d(:,3),'--', t, u(:,3));
legend(dict.desired_leg, dict.actual_leg);
ylim(calc_ylim([u_d(:,3);u(:,3)],.1));
ylabel('$u_3$ [N.m]');

subplot(4,1,4);
plot(t,u_d(:,4),'--', t, u(:,4));
legend(dict.desired_leg, dict.actual_leg);
ylim(calc_ylim([u_d(:,4);u(:,4)],.1));
ylabel('$u_4$ [N.m]');
xlabel(dict.time_label,'Interpreter','tex','FontSize',12);

end

