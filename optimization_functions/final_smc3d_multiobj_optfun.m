function F = final_smc3d_multiobj_optfun(cparam, physics_p, control_p, traj_p, sim_p)
% control_p.lambda_zpsi = [cparam(1); cparam(2)];
% control_p.kappa_zpsi = [0; 0];
% control_p.eta_zpsi = zeros(2,1)+0;
% 
% control_p.lambda_xtheta = [cparam(3); cparam(4)];
% control_p.lambda_xtheta_dot = [cparam(5); cparam(6)];
% control_p.kappa_xtheta = 0;
% control_p.eta_xtheta = 0+0;
% 
% control_p.lambda_yphi = [cparam(7); cparam(8)];
% control_p.lambda_yphi_dot = [cparam(9); cparam(10)];
% control_p.kappa_yphi = 0;
% control_p.eta_yphi = 0+0;
    control_p.lambda_zpsi = [5; 2];
    control_p.kappa_zpsi = [2; 2];
    control_p.eta_zpsi = zeros(2,1)+0;
    
    control_p.lambda_xtheta = [cparam(1); cparam(2)];
    control_p.lambda_xtheta_dot = [cparam(3); cparam(4)];
    control_p.kappa_xtheta = 2;
    control_p.eta_xtheta = 0;
%     
    control_p.lambda_yphi = [-cparam(1); cparam(2)];
    control_p.lambda_yphi_dot = [-cparam(3); cparam(4)];
    control_p.kappa_yphi = 2;
%     control_p.lambda_yphi = [cparam(7); cparam(8)];
%     control_p.lambda_yphi_dot = [cparam(9); cparam(10)];
%     control_p.kappa_yphi = 10;
    
    control_p.eta_yphi = 0;


theta_ratio = control_p.lambda_xtheta(2)/control_p.lambda_xtheta_dot(2);
x_ratio = control_p.lambda_xtheta(1)/control_p.lambda_xtheta_dot(1);

if  theta_ratio <= x_ratio
    F = [1000 1000];
    return;
end

phi_ratio = control_p.lambda_yphi(2)/control_p.lambda_yphi_dot(2);
y_ratio = control_p.lambda_yphi(1)/control_p.lambda_yphi_dot(1);

if  phi_ratio <= y_ratio
    F = [1000 1000];
    return;
end

try
    [t,q] = ode15s(@(t,x) ode_fun(t,x,physics_p,control_p,traj_p,sim_p), sim_p.t, sim_p.x0);
catch ME
    disp(ME.identifier);
    F = [1000 1000];
    return;
end

if size(sim_p.qd,1) ~= size(q,1)
    F = [1000 1000];
    return;
end

% phiL = q(:, 7);
% thetaL = q(:, 8);
% 
% p = [-sin(thetaL), sin(phiL).*cos(thetaL), -cos(phiL).*cos(thetaL)];
% 
% xLd = sim_p.qd(:, 1:3) + physics_p.l*p;
% xL = q(:, 1:3) + physics_p.l*p;
% 
% 
% e = xLd - xL;
e = q(:, 1:3) - sim_p.qd(:, 1:3);
etheta = q(t>3.75, 4:5) - sim_p.qd(t>3.75, 4:5);
% e(:, 2:4) = 0;

F = [trace(e'*e), trace(etheta'*etheta)];



end

