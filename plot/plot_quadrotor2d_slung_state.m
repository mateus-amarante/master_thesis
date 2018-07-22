function plot_quadrotor2d_slung_state(t,q,qd,u,physics_p)

r = physics_p.r;
x = q(:,1);
z = q(:,2);
theta = q(:,3);
alpha = q(:,4);

xd = qd(:,1);
zd = qd(:,2);

figure;
plot(t,[x z theta alpha],t,[xd zd]);
legend('x','z','\theta','\alpha','x_d','z_d');
title('Full State + Full Desired Trajectory');
xlabel('Time [s]');

figure;
plot(t,u);
legend('F','U');
title('Control Input');
xlabel('Time [s]');

end