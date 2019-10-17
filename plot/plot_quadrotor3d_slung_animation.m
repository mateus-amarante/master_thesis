function plot_quadrotor3d_slung_animation(t,q,physics_p,traj_p)

% Compute desired trajectory
qd = traj_p.sample_fun(t);

% Physical Parameters
r = physics_p.r;
L = physics_p.L;
r_l = physics_p.load_radius;

% Robot State
x = q(:,1);
y = q(:,2);
z = q(:,3);

% phi   = q(:,4);
% theta = q(:,5);
% psi   = q(:,6);

xyz = q(:,1:3);
rpy = q(:,4:6);

phiL   = q(:,7);
thetaL = q(:,8);

xyz_load = zeros(length(phiL), 3);

for i=1:length(phiL)
    % TODO: chedk this transformation
    xyz_load(i,1:3) = xyz(i,:) + (eul2rotm([phiL(i) thetaL(i) 0],'XYZ')*[0; 0; -L])';
end

xyz_d = qd(:,1:3);

dt = t(2)-t(1);

offset_xy = .9*L;
offset_z = 2*r;
xlim_values = [min(xyz_load(:,1))-offset_xy, max(xyz(:,1))+offset_xy];
ylim_values = [min(xyz_load(:,2))-offset_xy, max(xyz(:,2))+offset_xy];
zlim_values = [min(xyz_load(:,3))-offset_z,  max(xyz(:,3))+offset_z];

xd=xyz_d(:,1);
yd=xyz_d(:,2);
zd=xyz_d(:,3);


[xe, ye, ze] = sphere();
xe = xe*r_l; ye = ye*r_l; ze = ze*r_l;

figure('units','normalized','outerposition',[0 0 1 1]);
grid on;
view(3);

load_sphere = surf(xe*eps, ye*eps, ze*eps);
load_sphere.EdgeColor = 'none';
load_sphere.FaceLighting = 'gouraud';
shading interp;

axis equal;
xlim(xlim_values);
ylim(ylim_values);
zlim(zlim_values);
% axis equal;

line(xd,yd,zd,'Color','g');

q_line = line(x(1),y(1),z(1),'Color','r');
qL_line = line(xyz_load(1,1),xyz_load(1,1),xyz_load(1,1),'Color','m','LineWidth',1);

drone_line_13 = line(x(1),x(1),x(1),'Color','k','LineWidth',1);
drone_line_24 = line(x(1),x(1),x(1),'Color','k','LineWidth',1);

cable_line = line(x(1),x(1),x(1),'Color','k','LineWidth',1);

% v = VideoWriter('peaks.avi','MPEG-4');
% v.FrameRate = 50;
% open(v);

for i=1:length(t)
    tic;
    
    pos = (xyz(i,:))';
    ang = (rpy(i,:));
    R = eul2rotm(flip(ang));
    
    pos_load = (xyz_load(i,:))';
    
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
    
    qL_line.XData = xyz_load(1:i,1);
    qL_line.YData = xyz_load(1:i,2);
    qL_line.ZData = xyz_load(1:i,3);
        
    drone_line_13.XData = rotors13_x;
    drone_line_13.YData = rotors13_y;
    drone_line_13.ZData = rotors13_z;
    
    drone_line_24.XData = rotors24_x;
    drone_line_24.YData = rotors24_y;
    drone_line_24.ZData = rotors24_z;
    
    cable_line.XData = [pos(1) pos_load(1)];
    cable_line.YData = [pos(2) pos_load(2)];
    cable_line.ZData = [pos(3) pos_load(3)];
    
    load_sphere.XData = xe + pos_load(1);
    load_sphere.YData = ye + pos_load(2);
    load_sphere.ZData = ze + pos_load(3);
    
%     frame = getframe(gcf);
%     writeVideo(v,frame);
    
    pause(dt-toc);
    
end

% close(v);

end