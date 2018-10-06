function M = calc_polysegment_matrix(n_deriv, t)
% Compute the transformation matrix of polynomial trajectories

% n_deriv: target derivative order + 1 (n_deriv = 1 for position-only)

% M =
% [t1^0 t1^1 t1^2 ... t1^(n_deriv*2); % Deriv 0, Time 1
% [t2^0 t2^1 t2^2 ... t2^(n_deriv*2); % Deriv 0, Time 2
%  ...  ...  ...  ... ...
%   0   t0   2*t0 ... (n_deriv*2)*t0^(n_deriv*2-1); % Deriv 1, Time 1
%   0   t1   2*t1  .. (n_deriv*2)*t1^(n_deriv*2-1); % Deriv 1, Time 2
%  ...  ...  ...  ... ... ]
%
% Polynomial exponent per column
% Derivative order per line

t = t(:);

poly_order = n_deriv*2 - 1;
exponents = poly_order:-1:0;

M_part = ones(1, n_deriv*2);

M = zeros(n_deriv*length(t), n_deriv*2);

for i=1:n_deriv
    M((i-1)*length(t)+1:i*length(t),1:end-i+1) = M_part.*(t.^exponents(i:end));    
    M_part = polyder(M_part);
end

M = fliplr(M);

end
