function coef = calc_polysegment_coef(tspan, q0, qf, n_vars, n_deriv_out)
% Plan polynomial trajectories for 'ncols' variables

% tspan = [t0 tf]
% q0 = [pos1 pos2 ... vel1 vel2 ... acc1 acc2 ...] # Initial State
% qf = [pos1 pos2 ... vel1 vel2 ... acc1 acc2 ...] # Final State
% n_vars: number of variables
% n_deriv_out: number of derivatives to output + 1(n_deriv_out = 1 for position-only output)

% Input anguments analysis
n_deriv_in = length(q0)/n_vars;

if nargin < 4 || isempty(n_deriv_out)
    n_deriv_out = n_deriv_in;
end

n_deriv_out = max([n_deriv_in, n_deriv_out]);

q0 = reshape(q0(:), n_vars, n_deriv_in)'; 
qf = reshape(qf(:), n_vars, n_deriv_in)';

qq = zeros(n_deriv_in*2, n_vars);
qq(1:2:end, :) = q0;
qq(2:2:end, :) = qf;

M = calc_polysegment_matrix(n_deriv_in, tspan);

coef = zeros(n_deriv_out*2, n_vars);

coef(1:n_deriv_in*2, :) = M\qq;

end
