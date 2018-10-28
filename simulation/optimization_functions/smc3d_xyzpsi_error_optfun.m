function F = smc3d_xyzpsi_error_optfun(cparam, physics_p, control_p, traj_p)


control_p.lambda_zpsi = [cparam(1); cparam(2)];
control_p.kappa_zpsi = [cparam(3); cparam(4)];
control_p.eta_zpsi = zeros(2,1);

control_p.lambda_xtheta = [cparam(5); cparam(6)];
control_p.lambda_xtheta_dot = [cparam(7); cparam(8)];
control_p.kappa_xtheta = cparam(9);
control_p.eta_xtheta = 0;

control_p.lambda_yphi = [-cparam(10); cparam(11)];
control_p.lambda_yphi_dot = [-cparam(12); cparam(13)];
control_p.kappa_yphi = cparam(14);
control_p.eta_yphi = 0;


try
    [~,x] = ode15s(@(t,x) ode_fun(t,x,physics_p,control_p,traj_p), traj_p.t, traj_p.x0);
catch ME
    disp(ME.identifier);
    F = 10^9;
    return;
end

% qd = [traj_p.qd(:,1:4)];%traj_p.sample_fun(t)'; % TODO: build trajectory before (in traj_p construction)
% q = x(:,[1:3, 6]);

qd = [traj_p.qd(:,1:4) zeros(size(traj_p.qd, 1), 2)];%traj_p.sample_fun(t)'; % TODO: build trajectory before (in traj_p construction)
q = x(:,[1:3, 6 12 13]);


if size(qd,1) ~= size(q,1)
    F = 10^9;
    return;
end

e = qd-q;
% e(:, 2:4) = 0;

F = trace(e'*e);

end