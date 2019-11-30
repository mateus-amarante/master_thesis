function plot_horizontal_state(t, xy, phitheta, phithetaL, dict)

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
legend(dict.desired_leg, dict.actual_leg);
th = title(dict.quad_position_title, 'Interpreter','tex','FontSize',12,'FontAngle','Italic','FontWeight','normal');
set(th,'Unit','normalized','Position',[.5, 1.1, 0]);
ylim(ylim_pos);
xlim([t(1) t(end)]);

subplot(2,3,4);
plot(t, zeros(size(t)), '--', t,y);
ylabel('$y$ [m]');
legend(dict.desired_leg, dict.actual_leg);
ylim(ylim_pos);
xlim([t(1) t(end)]);
xlabel(dict.time_label,'Interpreter','tex','FontSize',12);


subplot(2,3,2);
plot(t, zeros(size(t)), '--', t,phi);
ylabel('$\phi$ [rad]');
legend(dict.desired_leg, dict.actual_leg);
th = title(dict.quad_orientation_title, 'Interpreter','tex','FontSize',12,'FontAngle','Italic','FontWeight','normal');
set(th,'Unit','normalized','Position',[.5, 1.1, 0]);
ylim(ylim_ang);
xlim([t(1) t(end)]);

subplot(2,3,5);
plot(t, zeros(size(t)), '--', t,theta);
ylabel('$\theta$ [rad]');
legend(dict.desired_leg, dict.actual_leg);
ylim(ylim_ang);
xlim([t(1) t(end)]);
xlabel(dict.time_label,'Interpreter','tex','FontSize',12);

subplot(2,3,3);
plot(t, zeros(size(t)), '--', t,phiL);
ylabel('$\phi_L$ [rad]');
legend(dict.desired_leg, dict.actual_leg);
th = title(dict.cable_orientation_title, 'Interpreter','tex','FontSize',12,'FontAngle','Italic','FontWeight','normal');
set(th,'Unit','normalized','Position',[.5, 1.1, 0]);
ylim(ylim_ang);
xlim([t(1) t(end)]);

subplot(2,3,6);
plot(t, zeros(size(t)), '--', t,thetaL);
ylabel('$\theta_L$ [rad]');
legend(dict.desired_leg, dict.actual_leg);
ylim(ylim_ang);
xlim([t(1) t(end)]);
xlabel(dict.time_label,'Interpreter','tex','FontSize',12);

end

