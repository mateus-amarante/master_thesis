function plot_quadrotor3d(xyz,rpy,xyz_load,arm_length,rotor_radius,load_radius)

[xe, ye, ze] = sphere();
xe = xe*load_radius; ye = ye*load_radius; ze = ze*load_radius;
hold on;
for i=1:size(xyz,1)
  
    pos = (xyz(i,:))';
    ang = (rpy(i,:));
    R = eul2rotm(flip(ang));
    
    pos_load = (xyz_load(i,:))';
    
    rotor1_pos = pos + R*[ arm_length;  0; 0];
    rotor2_pos = pos + R*[ 0;  arm_length; 0];
    rotor3_pos = pos + R*[-arm_length;  0; 0];
    rotor4_pos = pos + R*[ 0; -arm_length; 0];
    
    rotors13_x = [rotor1_pos(1) rotor3_pos(1)];
    rotors13_y = [rotor1_pos(2) rotor3_pos(2)];
    rotors13_z = [rotor1_pos(3) rotor3_pos(3)];
    
    rotors24_x = [rotor2_pos(1) rotor4_pos(1)];
    rotors24_y = [rotor2_pos(2) rotor4_pos(2)];
    rotors24_z = [rotor2_pos(3) rotor4_pos(3)];
    
    normal = R(:,3)';

    rotor1_circle_points = plot_circle_3d(rotor1_pos', normal, rotor_radius);
    rotor2_circle_points = plot_circle_3d(rotor2_pos', normal, rotor_radius);
    rotor3_circle_points = plot_circle_3d(rotor3_pos', normal, rotor_radius);
    rotor4_circle_points = plot_circle_3d(rotor4_pos', normal, rotor_radius);

    % plot rotors
    line(rotor1_circle_points(1,:),rotor1_circle_points(2,:),rotor1_circle_points(3,:),'Color','k','LineWidth',1);
    line(rotor2_circle_points(1,:),rotor2_circle_points(2,:),rotor2_circle_points(3,:),'Color','k','LineWidth',1);
    line(rotor3_circle_points(1,:),rotor3_circle_points(2,:),rotor3_circle_points(3,:),'Color','k','LineWidth',1);
    line(rotor4_circle_points(1,:),rotor4_circle_points(2,:),rotor4_circle_points(3,:),'Color','k','LineWidth',1);

    % plot drone lines
    line(rotors13_x,rotors13_y,rotors13_z,'Color','k','LineWidth',1);
    line(rotors24_x,rotors24_y,rotors24_z,'Color','k','LineWidth',1);

    % plot cable line
    line([pos(1) pos_load(1)], [pos(2) pos_load(2)],[pos(3) pos_load(3)],'Color','k','LineWidth',1);

    % plot load
    load_sphere = surf(gca,xe + pos_load(1), ye + pos_load(2), ze + pos_load(3));
%     shading interp;
    load_sphere.EdgeColor = 'none';
    load_sphere.FaceLighting = 'gouraud';
    load_sphere.FaceColor = [.5 .5 .5];
    
end
hold off;
end