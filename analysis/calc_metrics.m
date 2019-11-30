function metrics = calc_metrics(t,x,qd,u,Fs,stop_time)
%% Metrics
rpy = x(:, 4:6);
beta = acos(cos(rpy(:,1)).*cos(rpy(:,2)));
beta_rms = rms(beta,1);

pqr = size(length(t),3);

for i=1:length(t)
    pqr(i,1:3) = (Ti2b(x(i,4),x(i,5))*x(i, 12:14)')';
end
% pqr_rms = rms(vecnorm(pqr,2,2),1);

phithetaL = x(t>=stop_time, [7,8]);
alpha = acos(cos(phithetaL(:,1)).*cos(phithetaL(:,2)));
alpha_rms = rms(alpha,1);
% alpha_max = max(alpha);

xyz_rms = norm(rms(x(:,1:3) - qd(:,1:3),1));
% u_power = [power_u1, power_u2, power_u3, power_u4];
% tau_rms = rms(vecnorm(u(:,2:4),2,2),1)/100;
% u1_rms = rms(u(:,1),1)/100;

meanfreq_pqr = meanfreq(vecnorm(pqr,2,2),Fs);

metrics = [xyz_rms, beta_rms, alpha_rms, meanfreq_pqr];

end

