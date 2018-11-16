function plot_quadrotor2d_slung_animation(t,q,qd,physics_p)

r = physics_p.r;
L = physics_p.L;
r_l = physics_p.load_radius;

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


xlim(xlim_values);
ylim(ylim_values);
axis equal;

line(xd,zd,'Color','g'); % Desired Trajectory

q_line = line(x(1),x(1),'Color','r'); % Drone COG Trajectory
drone_line = line(x(1),x(1),'Color','k','LineWidth',1); % Drone arms
cable_line = line(x(1),x(1),'Color','k'); % Cable

load_rect = rectangle('Position',ones(1,4)*x(1),'Curvature',[1 1],'FaceColor',[.5 .5 .5]);

% v = VideoWriter('peaks.avi');
% open(v);

tic;
for i=1:length(t)

    q_line.XData = x(1:i);
    q_line.YData = z(1:i);

    drone_line.XData = [x(i) x(i)] + r*[ cos(theta(i)) -cos(theta(i))];
    drone_line.YData = [z(i) z(i)] + r*[-sin(theta(i))  sin(theta(i))];
    
    cable_line.XData = [x(i) xload(i)];
    cable_line.YData = [z(i) zload(i)];
    
    load_rect.Position = [xload(i)-r_l,zload(i)-r_l,2*r_l,2*r_l];

%     frame = getframe(gcf);
%     writeVideo(v,frame);
    
    pause(dt-toc);
    tic;
    
end

% close(v);

end