function plot_sliding_variables(t, s, dict)


%% Plot Robot Orientation and Euler Angles Rate
ylim_s = calc_ylim(s,.1);

figure;
% set(gcf, 'OuterPosition', [300, 150, 940, 770]);
% subplot(3,2,1);
plot(t,s);
ylabel(dict.sliding_variables_title,'Interpreter','tex','FontSize',12);
xlabel(dict.time_label,'Interpreter','tex','FontSize',12);
legend('$s_1$','$s_2$','$s_3$','$s_4$');
ylim(ylim_s);

end

