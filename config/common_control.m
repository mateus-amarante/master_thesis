function control_p = common_control(cparam)

    control_p.lambda_zpsi = [cparam(1); cparam(2)];
    control_p.kappa_zpsi = [cparam(3); cparam(4)];
    control_p.eta_zpsi = zeros(2,1)+.5;
    
    control_p.lambda_xtheta = [cparam(5); cparam(6)];
    control_p.lambda_xtheta_dot = [cparam(7); cparam(8)];
    control_p.kappa_xtheta = cparam(9);
    control_p.eta_xtheta = .5;
    
    control_p.lambda_yphi = [cparam(10); cparam(11)];
    control_p.lambda_yphi_dot = [cparam(12); cparam(13)];
    control_p.kappa_yphi = cparam(14);
    control_p.eta_yphi = .5;
    
    control_p.n_inputs = 4;

end