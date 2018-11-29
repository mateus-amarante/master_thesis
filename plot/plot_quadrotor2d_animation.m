function plot_quadrotor2d_animation(t,q,qd,physics_p)
   
r = physics_p.r;
x = q(:,1);
z = q(:,2);
theta = q(:,3);

xd = qd(:,1);
zd = qd(:,2);

figure('units','normalized','outerposition',[0 0 1 1]);

dt = t(2)-t(1);
xlim_values = [min(x)-2*r max(x)+2*r];
ylim_values = [min(z)-2*r max(z)+2*r];
xlim(xlim_values);
ylim(ylim_values);
axis equal

line(xd,zd,'Color','g');

q_line = line(x(1),x(1),'Color','r');
drone_line = line(x(1),x(1),'Color','k','LineWidth',1);

tic;
for i=1:length(t)
    
    q_line.XData = x(1:i);
    q_line.YData = z(1:i);

    drone_line.XData = [x(i) x(i)] + r*[ cos(theta(i)) -cos(theta(i))];
    drone_line.YData = [z(i) z(i)] + r*[-sin(theta(i))  sin(theta(i))];

    pause(dt-toc);
    tic;
    
end

end