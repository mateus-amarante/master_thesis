function plot_relative_error(t,xy,xy_d,rpy)
figure;

x = xy(:,1);
y = xy(:,2);

xd = xy_d(:,1);
yd = xy_d(:,2);

phi = rpy(:,1);
theta = rpy(:,2);
psi = rpy(:,3);

xb = cos(psi).*(xd-x) + sin(psi).*(yd-y);
yb = -sin(psi).*(xd-x) + cos(psi).*(yd-y);

figure;
subplot(2,2,1);
plot(t,xb);

subplot(2,2,2);
plot(t,theta);

subplot(2,2,3);
plot(t,yb);

subplot(2,2,4);
plot(t,phi);


end

