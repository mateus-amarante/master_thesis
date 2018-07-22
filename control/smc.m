function [u s ueq usw] = smc(e,edot,xddot_d,f,b,lambda,kappa,eta)
% First Order Sliding-Mode Control of Fully-actuated Systems

s = lambda.*e + edot;

ueq = (lambda.*edot + xddot_d - f)./b;

ueq(find(b==0))=0;

usw = (kappa.*s + eta.*sign(s))./b;

usw(find(b==0))=0;

u = ueq + usw;

end