function [u, s, ueq, usw, s_aux] = smcu(e,xddot_d,f,b,lambda,kappa,eta, switch_fun)
% First Order Sliding-Mode Control of Underactuated Systems

if nargin < 8
    switch_fun = @(x) sign(x);
end

% Variable renaming
edot = e(end/2+1:end); %must come first
e = e(1:end/2);

lambdadot = lambda(end/2+1:end); % must come first
lambda = lambda(1:end/2);


% Sliding variable
s = lambda'*e + lambdadot'*edot;

s_aux = edot + lambda./lambdadot.*e;

% u calculation
den = lambdadot'*b;

ueq = (lambda'*edot + lambdadot'*(xddot_d-f))/den;

usw = (kappa*s + eta*switch_fun(s))/den;

u = ueq + usw;
end