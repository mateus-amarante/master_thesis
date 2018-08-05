% function plot_quadrotor2d_animation(t,r,xd,zd,x,z,theta)
function plot_quadrotor2d_animation(t,q,qd,physics_p)
   
r = physics_p.r;
x = q(:,1);
z = q(:,2);
theta = q(:,3);

xd = qd(:,1);
zd = qd(:,2);

figure('units','normalized','outerposition',[0 0 1 1]);
% axis equal;

dt = t(2)-t(1);
xlim_values = [min(x)-2*r max(x)+2*r];
ylim_values = [min(z)-2*r max(z)+2*r];

for i=1:length(t)
    rotor_x = [x(i) x(i)] + r*[ cos(theta(i)) -cos(theta(i))];
    rotor_z = [z(i) z(i)] + r*[-sin(theta(i))  sin(theta(i))];
    
    tic;
    plot(xd,zd,'g-',x(i),z(i),'*k',rotor_x,rotor_z,'b-');
    xlim(xlim_values);
    ylim(ylim_values);
    axis equal
    time = toc;
    pause(dt-time);
    
end

end