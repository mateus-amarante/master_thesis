function F = smc2d_xzerror_optfun(x,physics_p,control_p,traj_p)


control_p.lambda_z = x(1);

control_p.lambda_x = x(2);
control_p.lambda_theta = x(3);

control_p.lambda_xdot = x(4);
control_p.lambda_thetadot = x(5);

control_p.kappa_z = x(6);
control_p.eta_z = 0;

control_p.kappa_xtheta = x(7);
control_p.eta_xtheta = 0;

try
    [~,x] = ode45(@(t,x) ode_fun(t,x,physics_p,control_p,traj_p), traj_p.t, traj_p.x0);
catch ME
    disp(ME.identifier);
    F = 1000;
    return;
end

qd = traj_p.qd(:,1:4);%traj_p.sample_fun(t)'; % TODO: build trajectory before (in traj_p construction)
q = x(:,1:2);
qdot = x(:,end/2+1:end/2+2);

if size(qd,1) ~= size(q,1)
    F = 10^9;
    return;
end

exz = qd-[q,qdot];

F = trace(exz'*exz);

end