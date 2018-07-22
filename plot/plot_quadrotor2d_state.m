function plot_quadrotor2d_state(t,q,qd,u,physics_p)

r = physics_p.r;
x = q(:,1);
z = q(:,2);
theta = q(:,3);

xd = qd(:,1);
zd = qd(:,2);

figure;
plot(t,[x z theta],t,[xd zd]);
legend('x','z','\theta','x_d','z_d');
title('Full State + Full Desired Trajectory');
xlabel('Time [s]');

figure;
plot(t,u);
legend('F','U');
title('Control Input');
xlabel('Time [s]');
end