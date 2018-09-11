% function plot_quadrotor2d_slung_animation(t,r,L,xd,zd,x,z,theta,xload,zload)
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

% v = VideoWriter('peaks.avi');
% open(v);

for i=1:length(t)
    rotor_x = [x(i) x(i)] + r*[ cos(theta(i)) -cos(theta(i))];
    rotor_z = [z(i) z(i)] + r*[-sin(theta(i))  sin(theta(i))];
    
    cla
    tic;
%     plot(xd,zd,'g-',[x(i) xload(i)],[z(i) zload(i)],'*k-',rotor_x,rotor_z,'b-');
    
    line(xd,zd,'Color','g'); % Desired Trajectory
    line(x(1:i),z(1:i),'Color','r'); % Drone COG Trajectory
    line(rotor_x,rotor_z,'Color','k','LineWidth',1); % Drone arms
    line([x(i) xload(i)],[z(i) zload(i)],'Color','k'); % Cable
    
    load_pos = [xload(i)-r_l,zload(i)-r_l,2*r_l,2*r_l];
    rectangle('Position',load_pos,'Curvature',[1 1],'FaceColor',[.5 .5 .5]); % Load
    

    
%     frame = getframe(gcf);
%     writeVideo(v,frame);
    
%     time = toc;
    pause(dt-toc);
    
end

% close(v);

end