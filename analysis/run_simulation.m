function [t,x,qd,u,metrics] = run_simulation(physics_p, control_p, traj_p, sim_p)
[t,x] = ode45(@(t, x) ode_fun(t, x, physics_p, control_p, traj_p, sim_p), sim_p.t, sim_p.x0);

%% Input trajectory
qd = traj_p.sample_fun(t);

%% Control input
u = zeros(length(t), control_p.n_inputs);
s = u;
ueq = u;
usw = u;

for i=1:length(t)
    [u(i,:), s(i,:), ueq(i,:), usw(i,:)] = control_p.control_fun(x(i,:),qd,physics_p,control_p);
end

%% Metrics
rpy = x(:, 4:6);
rpy_rms = rms(rpy,1);
beta = acos(cos(rpy(:,1)).*cos(rpy(:,2)));
beta_rms = rms(beta,1);

% pqr_rms = (Ti2b(rms_rpy(1),rms_rpy(2))*rms_rpy(4:6)')';

pqr = size(length(t),3);

for i=1:length(t)
    pqr(i,1:3) = (Ti2b(x(i,4),x(i,5))*x(i, 12:14)')';
end
pqr_rms = rms(vecnorm(pqr,2,2),1);

% rms_phithetaL = rms(x(t>=traj_p.stop_time, [7,8,15,16]),1);
% phithetaL_rms = rms(x(t>=traj_p.stop_time, [7,8]),1);
phithetaL_rms = rms(x(:, [7,8]),1);
% rms_phithetaL = [rms_phithetaL acos(cos(rms_phithetaL(1))*cos(rms_phithetaL(2)))];

phithetaL = x(t>=traj_p.stop_time, [7,8]);
alpha = acos(cos(phithetaL(:,1)).*cos(phithetaL(:,2)));
alpha_rms = rms(alpha,1);
% alpha_max = max(alpha);


u_std = std(u,1)/10;

xyz_rms = norm(rms(x(:,1:3) - qd(:,1:3),1));
xyzdot_rms = norm(rms(x(:,9:11) - qd(:,9:11),1));

% [meanfreq_u1,power_u1] = meanfreq(u(:,1), sim_p.Fs);
% [meanfreq_u2,power_u2] = meanfreq(u(:,2), sim_p.Fs);
% [meanfreq_u3,power_u3] = meanfreq(u(:,3), sim_p.Fs);
% [meanfreq_u4,power_u4] = meanfreq(u(:,4), sim_p.Fs);
% 
% u_freq = [meanfreq_u1, meanfreq_u2, meanfreq_u3, meanfreq_u4];
% u_power = [power_u1, power_u2, power_u3, power_u4];

tau_rms = rms(vecnorm(u(:,2:4),2,2),1)/100;
u1_rms = rms(u(:,1),1)/100;

% meanfreq_phi = meanfreq(x(:,4), sim_p.Fs);
% meanfreq_theta = meanfreq(x(:,5), sim_p.Fs);
% meanfreq_psi = meanfreq(x(:,6), sim_p.Fs);
% 
% meanfreq_phidot = meanfreq(pqr(:,1), sim_p.Fs);
% meanfreq_thetadot = meanfreq(pqr(:,2), sim_p.Fs);
% meanfreq_psidot = meanfreq(pqr(:,3), sim_p.Fs);
meanfreq_pqr = meanfreq(vecnorm(pqr,2,2),sim_p.Fs);
% u1_peaks = length(findpeaks(x(:,4)));
% u2_peaks = length(findpeaks(x(:,5)));
% u3_peaks = length(findpeaks(pqr(:,1)));
% u4_peaks = length(findpeaks(pqr(:,2)));

% u1_peaks = length(findpeaks(u(:,1)));
% u2_peaks = length(findpeaks(u(:,2)));
% u3_peaks = length(findpeaks(u(:,3)));
% u4_peaks = length(findpeaks(u(:,4)));


rpydot_peaks = length(findpeaks(x(:,12)))+length(findpeaks(x(:,13)));
rpydot_peaks = rpydot_peaks/100;

% u_peaks = sum([u1_peaks,u2_peaks, u3_peaks, u4_peaks])/100.;

% freq_rpy = [meanfreq_phi, meanfreq_theta, meanfreq_psi, meanfreq_phidot, meanfreq_thetadot, meanfreq_psidot];

% metrics = [rpy_rms, phithetaL_rms, alpha_rms, xyz_rms, pqr_rms, u_rms,rpydot_peaks];
metrics = [xyz_rms, beta_rms, alpha_rms, meanfreq_pqr];
end

