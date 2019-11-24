function [rms_rpy, freq_rpy, rms_phithetaL, u_std, s_rms] = calc_metrics(t,x,u,s,stop_time)

Fs = 1./(t(2)-t(1));

rms_rpy = rms(x(:, [4:6,12:14]),1);

meanfreq_phi = meanfreq(x(:,4), Fs);
meanfreq_theta = meanfreq(x(:,5), Fs);
meanfreq_psi = meanfreq(x(:,6), Fs);

meanfreq_phidot = meanfreq(x(:,12), Fs);
meanfreq_thetadot = meanfreq(x(:,13), Fs);
meanfreq_psidot = meanfreq(x(:,14), Fs);

freq_rpy = [meanfreq_phi, meanfreq_theta, meanfreq_psi, meanfreq_phidot, meanfreq_thetadot, meanfreq_psidot];

rms_phithetaL = rms(x(t>=stop_time, [7,8,15,16]),1);

s_rms = rms(s,1);
u_std = std(u,1);

end

