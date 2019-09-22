function sample_traj_fun = plan_polynomial_trajectory2(t, q, n_vars, n_deriv_out)

% q: [pos1 pos2 ... vel1 vel2 ... acc1 acc2 ...] for each time (row)
% n_vars: number of variables
% n_deriv_out: number of derivatives to output

% sample_traj_fun(t)
%   output: qd=[pos1 pos2 ... vel1 vel2 ... acc1 acc2 ...] for each time (row)

n_points = numel(t);

% Input anguments analysis
n_deriv_in = size(q,2)/n_vars-1;

if nargin < 4 || isempty(n_deriv_out)
    n_deriv_out = n_deriv_in;
    
    if nargin < 3 || isempty(n_vars)
        n_vars = n_deriv_in;
    end
end

% Reshape input (one variable per column)
v = zeros(n_points*(n_deriv_in+1), n_vars);

for i=1:n_vars
    v(:, i) = reshape(q(:, i + n_vars*(0:(n_deriv_in)))', n_points*(n_deriv_in+1), 1);
end
v = v'; % one variable per row

t = reshape(repmat(t,1,n_deriv_in+1)', 1, n_points*(n_deriv_in+1)); % repeat t n_deriv_in times
sp = spapi(augknt(t, n_deriv_in*2+2), t, v); % compute the spline

sample_traj_fun = @(t)sample(t, fnxtr(sp,1), n_vars, n_deriv_out);

    function v_out = sample(t, sp, n_vars, n_deriv_out) 
        v_out = zeros(numel(t), n_vars*(n_deriv_out+1));
        for der=0:n_deriv_out
            v_out(:,der*n_vars+1:(der+1)*n_vars) = fnval(fnder(sp,der), t)';
        end
    end
end

