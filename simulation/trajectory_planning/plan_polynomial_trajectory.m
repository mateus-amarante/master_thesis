function sample_traj_fun = plan_polynomial_trajectory(t, q, n_vars, n_deriv_out)

% q: [pos1 pos2 ... vel1 vel2 ... acc1 acc2 ...] for each time (row)
% n_vars: number of variables
% n_deriv_out: number of derivatives to output + 1(n_deriv_out = 1 for position-only output)

% sample_traj_fun(t)
%   output: qd=[pos1 pos2 ... vel1 vel2 ... acc1 acc2 ...] for each time (row)

n_points = length(t);

% Input anguments analysis
n_deriv_in = size(q,2)/n_vars;

if nargin < 4 || isempty(n_deriv_out)
    n_deriv_out = n_deriv_in;
    
    if nargin < 3 || isempty(n_vars)
        n_vars = n_deriv_in;
    end
end

n_deriv = max([n_deriv_in, n_deriv_out]);
coef_matrix = zeros(n_points-1, n_deriv*2*n_vars);

for i=1:n_points-1

    tspan = [t(i) t(i+1)];
    
    q0 = q(i,:); 
    qf = q(i+1,:);
    
    coef = calc_polysegment_coef(tspan, q0, qf, n_vars, n_deriv_out);
    
    coef_matrix(i,:) = coef(:)';
end

sample_traj_fun = @(tt) sample(t, tt, coef_matrix, n_deriv*2, n_vars, n_deriv_out);

    function qqd = sample(tq, t, coef_matrix, size_coef, n_vars, n_deriv_out)
        
        qqd = zeros(length(t), n_deriv_out*n_vars);

        t(t < tq(1))   = tq(1);
        t(t > tq(end)) = tq(end);
        
        for j=1:length(t)

            idx = find(tq >= t(j), 1)-1;
            
            if idx == 0
                idx = 1;
            end
            
            coefs = reshape(coef_matrix(idx,:), size_coef, []);
            qd_i = sample_polysegment(coefs, t(j));
            
            qqd(j,:) = qd_i(1:n_deriv_out*n_vars);
        end

    end

end