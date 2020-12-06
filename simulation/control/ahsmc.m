function [u,S,s,ueq,usw] = ahsmc(e,edot,xddot_d,f,b,lambda,alpha,kappa,eta)
% Aggregated Hierarchical Sliding-Mode Control of Underactuated Systems

% lambda = lambda(:)';
% alpha = alpha(:)';

s = lambda.*e + edot;
S = alpha'*s;

b(b==0)=1;

ueq = (-f + lambda.*edot + xddot_d)./b;

coef = alpha.*b;

u = (coef'*ueq + eta*sign(S) + kappa*S)/sum(coef);

usw = u - sum(ueq);

end