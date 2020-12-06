t = [0 3 6 9]'; % Waypoints times
tt = (t(1):.01:t(end))'; % Sample times

% Waypoints positions
pos = [0 0 0;
    0 0 0;
    1 1 0;
    1 1  0];

yaw = [0; 0; pi/2; pi/2];

physics_p = quadrotor3d_slung_physics();

sample_fun = waypoint_flat_trajectory(t, pos, yaw, physics_p);
flat_outputs = sample_fun(tt);
% plot(tt, qqd(:,1:3), tt, flat_outputs(:,1:3));
% plot(tt, flat_outputs(:,end-1:end));

traj_p.sample_fun = sample_fun;

plot_quadrotor3d_slung_flat_animation(tt, flat_outputs, physics_p, traj_p);

x = flat_outputs(:, 1);
y = flat_outputs(:, 2);
z = flat_outputs(:, 3);

phiL = flat_outputs(:, 7);
thetaL = flat_outputs(:, 8);

xL = x - physics_p.l * sin(thetaL);
yL = y + physics_p.l * sin(phiL) .* cos(thetaL);
zL = z - physics_p.l * cos(phiL) .* cos(thetaL);

% plot(tt,control_input);
% plot3(qqd(:, 1), qqd(:, 2), qqd(:, 3)); hold on;
plot3(x,y,z); hold on;
plot3(xL,yL,zL);

sample_rLd = waypoint_poly_trajectory(t, pos, 6);
sample_psid = waypoint_poly_trajectory(t, yaw, 2);

qqd = sample_rLd(tt);
yaw_tt = sample_psid(tt);

testCase = matlab.unittest.TestCase.forInteractiveUse;
assertEqual(testCase,qqd(:,1:3),[xL,yL,zL],'AbsTol',1e-12);
assertEqual(testCase,yaw_tt(:,1),flat_outputs(:,6),'AbsTol',1e-12);
assertEqual(testCase,yaw_tt(:,2),flat_outputs(:,14),'AbsTol',1e-12);
assertEqual(testCase,yaw_tt(:,3),flat_outputs(:,22),'AbsTol',1e-12);