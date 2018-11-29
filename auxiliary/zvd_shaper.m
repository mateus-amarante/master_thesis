function [us, ts, A] = zvd_shaper(t,u,wn,zeta)

damping = sqrt(1-zeta^2);
wd = damping*wn;
Td = 2*pi/wd;

K = exp(-zeta*pi/damping);

ts = [0 Td/2 Td]';

A1 = 1/(1+2*K+K^2);
A = [A1 2*A1 K*A1]';

[~, idx2] = min(abs(t-ts(2)));
[~, idx3] = min(abs(t-ts(3)));

a = zeros(idx3,1);
a(1) = A(1);
a(idx2) = A(2);
a(idx3) = A(3);

us = u;

for j=1:size(u,2)
    us(:,j) = conv(u(:,j),a,'same');
end

end