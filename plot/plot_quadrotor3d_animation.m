% function plot_quadrotor3d_animation(t,r,xyz_d,xyz,rpy)
function plot_quadrotor3d_animation(t,q,qd,physics_p)

r = physics_p.r;
xyz = q(:,1:3);
rpy = q(:,4:6);

x = xyz(:,1);
y = xyz(:,2);
z = xyz(:,3);

xyz_d = qd(:,1:3);
% psi_d = qd(:,4);

dt = t(2)-t(1);

xlim_values = [min(xyz(:,1))-2*r max(xyz(:,1))+2*r];
ylim_values = [min(xyz(:,2))-2*r max(xyz(:,2))+2*r];
zlim_values = [min(xyz(:,3))-2*r max(xyz(:,3))+2*r];

xd=xyz_d(:,1);
yd=xyz_d(:,2);
zd=xyz_d(:,3);

figure('units','normalized','outerposition',[0 0 1 1]);
xlim(xlim_values);
ylim(ylim_values);
zlim(zlim_values);
axis equal;
view(3);

line(xd,yd,zd,'Color','g');

q_line = line(x(1),y(1),z(1),'Color','r');

drone_line_13 = line(x(1),x(1),x(1),'Color','k','LineWidth',1);
drone_line_24 = line(x(1),x(1),x(1),'Color','k','LineWidth',1);

for i=1:length(t)
    pos = (xyz(i,:))';
    ang = (rpy(i,:));
    R = eul2rotm(flip(ang));
    
    rotor1_pos = pos + R*[ r;  0; 0];
    rotor2_pos = pos + R*[ 0;  r; 0];
    rotor3_pos = pos + R*[-r;  0; 0];
    rotor4_pos = pos + R*[ 0; -r; 0];
    
    rotors13_x = [rotor1_pos(1) rotor3_pos(1)];
    rotors13_y = [rotor1_pos(2) rotor3_pos(2)];
    rotors13_z = [rotor1_pos(3) rotor3_pos(3)];
    
    rotors24_x = [rotor2_pos(1) rotor4_pos(1)];
    rotors24_y = [rotor2_pos(2) rotor4_pos(2)];
    rotors24_z = [rotor2_pos(3) rotor4_pos(3)];
    
    q_line.XData = x(1:i);
    q_line.YData = y(1:i);
    q_line.ZData = z(1:i);
    
    drone_line_13.XData = rotors13_x;
    drone_line_13.YData = rotors13_y;
    drone_line_13.ZData = rotors13_z;
    
    drone_line_24.XData = rotors24_x;
    drone_line_24.YData = rotors24_y;
    drone_line_24.ZData = rotors24_z;
    
    pause(dt-toc);
    tic;
end

end