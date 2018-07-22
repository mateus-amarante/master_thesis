function [u s ueq usw] = usmc(e,xddot_d,f,b,lambda,kappa,eta)
% First Order Sliding-Mode Control of Underactuated Systems

% Variable renaming
edot = e(end/2+1:end);
e = e(1:end/2);

lambdadot = lambda(end/2+1:end);
lambda = lambda(1:end/2);

% Sliding variable
s = lambda'*e + lambdadot'*edot;

% u calculation
den = lambdadot'*b;

ueq = (lambda'*edot + lambdadot'*(xddot_d-f))/den;

usw = (kappa*s + eta*sign(s))/den;

u = ueq + usw;
end