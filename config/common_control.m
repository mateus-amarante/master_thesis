function control_p = common_control(cparam)

    if nargin < 1
%         cparam = [2,1,4,2];
        cparam = [2,1,5,.1];
    end
    
    control_p.lambda_zpsi = [2; 2];
    control_p.kappa_zpsi = [1; 1];
    control_p.eta_zpsi = zeros(2,1)+1;
    control_p.epsilon_zpsi = zeros(2,1)+50;
%     control_p.kappa_zpsi(1) = 100;
%     control_p.eta_zpsi(1) = 4;
    
    control_p.lambda_xtheta = [cparam(1); cparam(2)];
    control_p.lambda_xtheta_dot = [cparam(3); cparam(4)];
    control_p.kappa_xtheta = 2;
    control_p.eta_xtheta = 1;
    control_p.epsilon_xtheta = 50;

    control_p.lambda_yphi = [-cparam(1); cparam(2)];
    control_p.lambda_yphi_dot = [-cparam(3); cparam(4)];
    control_p.kappa_yphi = 2;  
    control_p.eta_yphi = 1;
    control_p.epsilon_yphi = 50;
    
    control_p.n_inputs = 4;

end