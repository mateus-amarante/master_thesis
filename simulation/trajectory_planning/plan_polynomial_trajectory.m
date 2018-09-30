function sample_traj_fun = plan_polynomial_trajectory(t,q,qdot,qddot)

n_vars = size(q,2);
n_points = length(t);
n_deriv = (nargin - 1);

qq = zeros(n_deriv*n_points, n_vars);

if n_deriv == 1 % LINEAR
    
    qq = q;
    
elseif n_deriv == 2 % CUBIC
    
    qq(1:2:end) = q;
    qq(2:2:end) = qdot;
    
elseif n_deriv == 3 % QUINTIC
    
    qq(1:3:end) = q;
    qq(2:3:end) = qdot;
    qq(3:3:end) = qddot;

end
% 
% coef_matrix = zeros(n_points, n_coefs*n_vars);
% 
% for i=1:n_points - 1
%     li = i*order;
%     ui = li + order - 1;
%     
%     lip1 = ui+1;
%     uip1 = ui + order;
%     
%     coefs = M\[qq(li:ui,:); qq(lip1:uip1, :)];
%     coefs = coefs(:)';
%     coef_matrix(i,:) = coefs;
% end

coef_matrix = zeros(n_points-1, n_deriv*2*n_vars);

for i=1:n_points-1

    tspan = [t(i) t(i+1)];
    
    lower_qq_i0 = (i - 1)*n_deriv + 1;
    upper_qq_i0 = i*n_deriv;
    
    q0 = qq(lower_qq_i0:upper_qq_i0,:);
    
    lower_qq_if = upper_qq_i0 + 1;
    upper_qq_if = upper_qq_i0 + n_deriv;
    
    qf = qq(lower_qq_if:upper_qq_if,:);
    
    coef = calc_polysegment_coef(tspan, q0, qf);
    
    coef_matrix(i,:) = coef(:)';
end

sample_traj_fun = @(tt) sample(t, tt, coef_matrix, n_deriv*2, n_vars);

    function qqd = sample(tq, t, coef_matrix, size_coef, n_vars)
        
        qqd = zeros(length(t), size_coef/2*n_vars);

        t(t < tq(1))   = tq(1);
        t(t > tq(end)) = tq(end);
        
        for j=1:length(t)

            idx = find(tq >= t(j), 1)-1;
            
            if idx == 0
                idx = 1;
            end
            
            coefs = reshape(coef_matrix(idx,:), size_coef, []);
            qqd(j,:) = sample_polysegment(coefs, t(j));
        end

    end

end