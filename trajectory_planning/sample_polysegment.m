function [qd] = sample_polysegment(coef,t)
% coef = [a0 a1 a2 a3 a4 a5]' per var (column) for 'n' variables
% a0 + a1*t + a2*t² + a3*t³ + a4*t? + a5*t?
% qd =[pos1 pos2 ... posn,
%      vel1 vel2 ... veln,
%      acc1 acc2 ... accn] for each time (row) for 'n' variables

n_var   = size(coef, 2);
n_coefs = size(coef, 1);
n_deriv = n_coefs/2;

M = calc_polysegment_matrix(n_deriv, t);

temp = M*coef;
% temp =        % One variable per column
% [p11 p21 ...; % Time 1
%  p12 p22 ...; % Time 2
%  ... ... ...; % ...
%  v11 v21 ...; % Time 1
%  v12 v22 ...; % Time 2
%  ... ... ...; % ...
%  a11 a21 ...; % Time 1
%  a12 a22 ...; % Time 2
%  ... ... ...] % ...

qd = zeros(length(t), n_var*n_deriv);

for i=1:n_deriv
    rows = (i-1)*length(t)+1:i*length(t);
    qd(:, (i-1)*n_var+1:i*n_var) = temp(rows, :);
end

end
