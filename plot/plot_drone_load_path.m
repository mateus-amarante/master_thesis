function plot_drone_load_path(x,qd,physics_p,title_str, dict)
phiL = x(:,7);
thetaL = x(:,8);

xyz = x(:,1:3);
xyz_d = qd(:,1:3);

xyz_load = zeros(length(phiL), 3);

for i=1:length(phiL)
    % TODO: chedk this transformation
    xyz_load(i,1:3) = xyz(i,:) + (eul2rotm([phiL(i) thetaL(i) 0],'XYZ')*[0; 0; -physics_p.l])';
end

offset_xy = .9*physics_p.l;
offset_z = 2*physics_p.r;
xlim_values = [min(xyz_load(:,1))-offset_xy, max(xyz(:,1))+offset_xy];
ylim_values = [min(xyz_load(:,2))-offset_xy, max(xyz(:,2))+offset_xy];
zlim_values = [min(xyz_load(:,3))-offset_z,  max(xyz(:,3))+offset_z];

fig = figure('units','normalized','outerposition',[0.25,0.2,0.46,0.56]);
set(fig,'Renderer','painters');
grid on;
view(3);

axis equal;
xlim(xlim_values);
ylim(ylim_values);
zlim(zlim_values);

line(qd(:,1),qd(:,2),qd(:,3),'Color','g');
line(qd(:,1),qd(:,2),qd(:,3)-physics_p.l,'Color','b');
line(xyz(:,1),xyz(:,2),xyz(:,3),'Color','r');
line(xyz_load(:,1),xyz_load(:,2),xyz_load(:,3),'Color','m');


xlabel('$x$ [m]','FontSize',14);
ylabel('$y$ [m]','FontSize',14);
zlabel('$z$ [m]','FontSize',14);
t = title(title_str,'FontSize',14);
% set(t,'Position',[5.5 0.4 1.00011]);
leg = legend(dict.quad_reference_trajectory_leg, dict.load_reference_trajectory_leg,...
    dict.quad_trajectory_leg, dict.load_trajectory_leg,"Interpreter","tex","FontSize",12);
set(leg,'FontSize',12);
end

