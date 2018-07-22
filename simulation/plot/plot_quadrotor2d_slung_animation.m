% function plot_quadrotor2d_slung_animation(t,r,L,xd,zd,x,z,theta,xload,zload)
function plot_quadrotor2d_slung_animation(t,q,qd,physics_p)

r = physics_p.r;
L = physics_p.L;

x = q(:,1);
z = q(:,2);
theta = q(:,3);
alpha = q(:,4);

xload = x - L*sin(alpha);
zload = z - L*cos(alpha);

xd = qd(:,1);
zd = qd(:,2);

figure('units','normalized','outerposition',[0 0 1 1]);

dt = t(2)-t(1);
xlim_values = [min(min(x)-2*r, min(xload)) max(max(x)+2*r, max(xload))];
ylim_values = [min(min(z)-2*r, min(zload)) max(max(z)+2*r, max(zload))];

for i=1:length(t)
    rotor_x = [x(i) x(i)] + r*[ cos(theta(i)) -cos(theta(i))];
    rotor_z = [z(i) z(i)] + r*[-sin(theta(i))  sin(theta(i))];
    
    tic;
    plot(xd,zd,'g-',[x(i) xload(i)],[z(i) zload(i)],'*k-',rotor_x,rotor_z,'b-');

    axis equal;
    xlim(xlim_values);
    ylim(ylim_values);
    
%     time = toc;
    pause(dt-toc);
    
end

end