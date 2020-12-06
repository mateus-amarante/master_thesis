load('all_18s.mat');
metrics_18 = metrics;
load('all_15s.mat');
metrics_15 = metrics;
load('all_12s.mat');
metrics_12 = metrics;
load('all_10s.mat');
metrics_10 = metrics;

figure;
subplot(2,2,1);
bar(metrics_18(:,3:4));
title('T = 18s', 'FontSize',14);
legend(strcat(dict.configuration_leg, " II"), strcat(dict.configuration_leg, " III"));
set(gca,'xticklabel',{'$r_{RMS}$','$\beta_{RMS}$','$\alpha_{RMS}$','$\bar{f}_\omega$'});

subplot(2,2,2);
bar(metrics_15(:,3:4));
title('T = 15s', 'FontSize',14);
legend(strcat(dict.configuration_leg, " II"), strcat(dict.configuration_leg, " III"));
set(gca,'xticklabel',{'$r_{RMS}$','$\beta_{RMS}$','$\alpha_{RMS}$','$\bar{f}_\omega$'});

subplot(2,2,3);
bar(metrics_12(:,3:4));
title('T = 12s');
legend(strcat(dict.configuration_leg, " II"), strcat(dict.configuration_leg, " III"));
set(gca,'xticklabel',{'$r_{RMS}$','$\beta_{RMS}$','$\alpha_{RMS}$','$\bar{f}_\omega$'});

subplot(2,2,4);
bar(metrics_10(:,3:4));
title('T = 10s', 'FontSize',14);
legend(strcat(dict.configuration_leg, " II"), strcat(dict.configuration_leg, " III"));
set(gca,'xticklabel',{'$r_{RMS}$','$\beta_{RMS}$','$\alpha_{RMS}$','$\bar{f}_\omega$'});