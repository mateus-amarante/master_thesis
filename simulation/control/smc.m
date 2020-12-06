function [u, s, ueq, usw] = smc(e,edot,xddot_d,f,b,lambda,kappa,eta, switch_fun)
% First Order Sliding-Mode Control of Fully-actuated Systems

if nargin < 9
    switch_fun = @(x) sign(x);
end

s = lambda.*e + edot;

ueq = (lambda.*edot + xddot_d - f)./b;

ueq(b==0)=0;

usw = (kappa.*s + eta.*switch_fun(s))./b;

usw(b==0)=0;

u = ueq + usw;

end