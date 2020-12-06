function plot_auxiliary_sliding_variables(t, s_aux, dict)


%% Plot Robot Orientation and Euler Angles Rate
ylim_s = calc_ylim(s_aux,.1);

figure;
% set(gcf, 'OuterPosition', [300, 150, 940, 770]);
subplot(2,1,1);
plot(t,s_aux(1:4));
% ylabel(dict.aux_sliding_variables_xtheta_title,'Interpreter','tex','FontSize',12);
xlabel(dict.time_label,'Interpreter','tex','FontSize',12);
legend('$s_x$','$s_{\theta}$');
ylim(ylim_s);

subplot(2,1,2);
plot(t,s_aux(5:8));
% ylabel(dict.aux_sliding_variables_yphi_title,'Interpreter','tex','FontSize',12);
xlabel(dict.time_label,'Interpreter','tex','FontSize',12);
legend('$s_y$','$s_{\phi}$');
ylim(ylim_s);


end