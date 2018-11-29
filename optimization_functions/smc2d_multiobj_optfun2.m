function F = smc2d_multiobj_optfun2(x,physics_p,control_p,traj_p)


control_p.lambda_z = x(1);
control_p.lambda_x = x(2);
control_p.lambda_theta = x(3);

control_p.kappa_z = x(4);
control_p.kappa_x = x(5);
control_p.kappa_theta = x(6);

try
    [~,x] = ode45(@(t,x) ode_fun(t,x,physics_p,control_p,traj_p), traj_p.t, traj_p.x0);
catch ME
    disp(ME.identifier);
    F = [10^9;10^9;10^9];
    return;
end

qd = traj_p.qd(:,1:2);%traj_p.sample_fun(t)'; % TODO: build trajectory before (in traj_p construction)
q = x(:,1:2);

if size(qd,1) ~= size(q,1)
    F = [10^9;10^9;10^9];
    return;
end

exz = qd-q;
F(1) = trace(exz'*exz);
F(2) = x(:,end-1)'*x(:,end-1);

exzdot = traj_p.qd(:, 3:4) - x(:, 5:6);
F(3) = trace(exzdot'*exzdot);

end