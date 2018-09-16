function u = nested3d_controller(q,qdot,xref,physics_p,control_p)
% https://www.youtube.com/watch?v=uPsenrdsu28
% q: [x z theta]'

% Params renaming
M = physics_p.M;
g = physics_p.g;
I = physics_p.I;
Ix = physics_p.Ix;
Iy = physics_p.Iy;
Iz = physics_p.Iz;
kt = physics_p.kt;
km = physics_p.km;
r = physics_p.r;
maxThrust = physics_p.maxThrust;

kp_xyz = control_p.kp_xyz;
kd_xyz = control_p.kd_xyz;
kp_rpy = control_p.kp_rpy;
kd_rpy = control_p.kd_rpy;

% Reference renaming
qd = xref(1:4);
qdot_d = xref(5:8);
qddot_d = xref(9:12);

xyz_d = qd(1:3);
xyzdot_d = qdot_d(1:3);
xyzddot_d = qddot_d(1:3);

psi_d = qd(4);
psidot_d = qdot_d(4);
psiddot_d = qddot_d(4);

% State renaming
xyz = q(1:3);
xyz_dot = qdot(1:3);
rpy = q(4:6);
rpy_dot = qdot(4:6);

phi = rpy(1);
theta = rpy(2);
psi = rpy(3);

% Control calculation
xyz_c = xyzddot_d + kd_xyz.*(xyzdot_d-xyz_dot) + kp_xyz.*(xyz_d-xyz);

xddot = g*(theta*cos(psi) + phi*sin(psi));
yddot = g*(theta*sin(psi) - phi*cos(psi));

xydot_c = kd_xyz(1:2).*(xyzddot_d(1:2)-[xddot;yddot]) + kp_xyz(1:2).*(xyzdot_d(1:2)-xyz_dot(1:2));

phi_c =   (xyz_c(1)*sin(psi_d) - xyz_c(2)*cos(psi_d))/g;
theta_c = (xyz_c(1)*cos(psi_d) + xyz_c(2)*sin(psi_d))/g;

phidot_c = xydot_c(1)*sin(psi_d) + xyz_c(1)*cos(psi_d)*psidot_d + ...
          -xydot_c(2)*cos(psi_d) + xyz_c(2)*sin(psi_d)*psidot_d;
thetadot_c = xydot_c(1)*cos(psi_d) - xyz_c(1)*sin(psi_d)*psidot_d + ...
             xydot_c(2)*sin(psi_d) + xyz_c(2)*cos(psi_d)*psidot_d;

rpy_c = [phi_c;theta_c;psi_d];
rpydot_c = [phidot_c;thetadot_c;psidot_d];

U = zeros(4,1);

U(1) = M*(g+xyz_c(3));
try
U(2:4) = kp_rpy.*(rpy_c-rpy) + kd_rpy.*(rpydot_c-rpy_dot);
catch ME
    ME
end

if U(1) > maxThrust
    U(1) = maxThrust;
end

u = U;

% Rotors velocities calculation
% M = [
%     kt*[1 1 1 1];
%     kt*r*[0 1 0 -1];
%     kt*r*[-1 0 1 0];
%     km*[1 -1 1 -1]];
% 
% u = sqrt(M\U);

end