% function plot_quadrotor3d_slung(t,r,xyz_d,xyz,rpy,xyz_load)
function plot_quadrotor3d_slung_animation(t,q,qd,physics_p)

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
offset = .5;
xlim_values = [min(xyz_load(:,1))-offset max(xyz(:,1))+offset];
ylim_values = [min(xyz_load(:,2))-offset max(xyz(:,2))+offset];
zlim_values = [min(xyz_load(:,3))-offset max(xyz(:,3))+offset];

xd=xyz_d(:,1);
yd=xyz_d(:,2);
zd=xyz_d(:,3);


[xe, ye, ze] = sphere();
xe = xe*r_l; ye = ye*r_l; ze = ze*r_l;

figure('units','normalized','outerposition',[0 0 1 1]);
grid on;
xlim(xlim_values);
ylim(ylim_values);
zlim(zlim_values);
axis equal;
view(3);
hold on;

for i=1:length(t)
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
    
    cla;
    s = surf(xe + pos_load(1), ye + pos_load(2), ze + pos_load(3));
    s.EdgeColor = 'none';
    s.FaceLighting = 'gouraud';
%     s.FaceColor = 'b';
    shading interp;
   
    tic;  
    line(xd,yd,zd,'Color','g');
    line(x(1:i),y(1:i),z(1:i),'Color','r');
    line(xyz_load(1:i,1),xyz_load(1:i,2),xyz_load(1:i,3),'Color','m');
    
    line(rotors13_x,rotors13_y,rotors13_z,'Color','k','LineWidth',2);
    line(rotors24_x,rotors24_y,rotors24_z,'Color','k','LineWidth',2);
    
    line([pos(1) pos_load(1)],[pos(2) pos_load(2)],[pos(3) pos_load(3)],'Color','k'); % Cable
    
    
    time = toc;
    pause(dt-time);
    
end

end