function [system_state, control_input] = differentially_flat_trajectory(flat_rL, flat_yaw, physics_p)

    % Physics parameters renaming
    M = physics_p.M;
    g = physics_p.g;
    I = physics_p.I;

    l = physics_p.l;
    m = physics_p.m;

    Cd = physics_p.Cd(:)'; % drag coefficients of the aircraft
%     cx = physics_p.cx;
%     cy = physics_p.cy;
%     cz = physics_p.cz;
    cL = physics_p.cL; % drag coefficients of the load

    % Auxiliar variables
    ez = [0, 0, 1];

    % Auxiliar functions
    rLvec = @(der) flat_rL(:, der * 3 + 1:(der + 1) * 3);
    yaw = @(der) flat_yaw(:, der + 1);
    
    % Cable tension determination
    Tp = @(der) -m * rLvec(der+2) - ~der* (m * g * ez) - cL*rLvec(der + 1);
    Tp_mat = [Tp(0) Tp(1) Tp(2) Tp(3) Tp(4)];
    Tp = @(der) Tp_mat(:, der * 3 + 1:(der + 1) * 3);
 
    pvec = Tp(0) ./ vecnorm(Tp(0), 2, 2);
    
    T = @(der) dot(Tp(der), pvec, 2);
    T_mat = [T(0) T(1) T(2) T(3) T(4)];
    T = @(der) T_mat(:, der + 1);

%     pvec = Tp(0) ./ T(0);
    pvec_dot = (Tp(1) - T(1).*pvec) ./ T(0);
    pvec_ddot = (Tp(2) - T(2).*pvec - 2*T(1).*pvec_dot) ./ T(0);
    pvec_3dot = (Tp(3) - T(3).*pvec - 3*T(2).*pvec_dot - 3*T(1).*pvec_ddot) ./ T(0);
    pvec_4dot = (Tp(4) - T(4).*pvec - 4*T(3).*pvec_dot - 6*T(2).*pvec_ddot - 4*T(1).*pvec_3dot) ./ T(0);  
    
    pvec_mat = [pvec, pvec_dot, pvec_ddot, pvec_3dot, pvec_4dot];
    p = @(der) pvec_mat(:, der * 3 + 1:(der + 1) * 3);
    
    % Compute load angle
    thetaL = asin(-pvec(:, 1));
    phiL = asin(pvec(:, 2) ./ cos(thetaL));

    % Compute load angular velocity/acceleration
    thetaLdot = -pvec_dot(:, 1) ./ cos(thetaL);
    phiLdot = (sin(phiL) .* sin(thetaL) .* thetaLdot + pvec_dot(:, 2)) ./ (cos(phiL) .* cos(thetaL));

    thetaLddot = (sin(thetaL) .* thetaLdot.^2 - pvec_ddot(:, 1)) ./ cos(thetaL);
    phiLddot = (sin(phiL) .* sin(thetaL) .* thetaLddot + 2 * cos(phiL) .* sin(thetaL) .* phiLdot .* thetaLdot + ...
        sin(phiL) .* cos(thetaL) .* (phiLdot.^2 + thetaL.^2) + pvec_ddot(:, 2)) ./ (cos(phiL) .* cos(thetaL));

    % Drone state
    rvec = @(der) rLvec(der) - l*p(der);

    % Rotation Matrix
    t = M * rvec(2) - Tp(0) + M * g * ez + Cd.*rvec(1);
    ezb = t ./ vecnorm(t, 2, 2);

    eyc = [-sin(yaw(0)), cos(yaw(0)), zeros(size(t, 1), 1)];
    
    aux = cross(eyc, ezb, 2);
    exb = aux ./ vecnorm(aux, 2, 2);
    eyb = cross(ezb, exb, 2);

    Rvec = [exb, eyb, ezb];
    
    % Euler angles
    rpy = zeros(size(ezb));

    for i = 1:size(Rvec, 1)
        rpy(i, :) = fliplr(rotm2eul(reshape(Rvec(i, :), [3, 3])));
    end
    phi = rpy(:, 1);
    theta = rpy(:, 2);
%     psi = rpy(:, 3);
    
    % TODO: Assert yaw input equals yaw output

    % Thrust force
    u1 = vecnorm(t, 2, 2);

    % Angular velocity
    u1dot = dot(M*rvec(3) + Cd.*rvec(2), ezb, 2);
    hw = (M*rvec(3) + Cd.*rvec(2) - u1dot .* ezb) ./ u1;

    p = -dot(hw, eyb, 2);
    q = dot(hw, exb, 2);
    r = (cos(theta).*yaw(1) - sin(phi).*q)./cos(phi);

    omega = [p, q, r];
    rpy_dot = rpy;
    for i = 1:size(Rvec, 1)
        Tbi = Tb2i(rpy(i, 1), rpy(i, 2));
        rpy_dot(i, :) = (Tbi * rpy(i, :)')';
    end
    phidot = rpy_dot(:, 1);
    thetadot = rpy_dot(:, 2);
%     psidot = rpy_dot(:, 3);

    % Angular acceleration
    aux = M*rvec(4) + Cd.*rvec(3) - cross(omega, cross(omega, u1.*ezb, 2), 2);
    u1ddot = dot(aux, ezb, 2);
    halpha = aux - u1ddot .* ezb - 2 * cross(omega, u1dot .* ezb, 2);

    pdot = -dot(halpha, eyb, 2);
    qdot = dot(halpha, exb, 2);
    rdot = (cos(theta).*yaw(2) - sin(phi).*qdot - thetadot.*(phidot + sin(theta).*yaw(1)))./cos(phi);

    omegadot = [pdot, qdot, rdot];

%     rpy_ddot = rpy;
%     for i = 1:size(Rvec, 1)
%         Tib = Ti2b(rpy(i, 1), rpy(i, 2));
%         Tbi = Tb2i(rpy(i, 1), rpy(i, 2));
%         
%         Tbi_dot = Tb2i_dot(rpy(i, 1), rpy(i, 2), omega(i, 1), omega(i, 2));
% 
%         rpy_dot(i, :) = (Tbi * rpy(i, :)')';
%         rpy_ddot(i, :) = (Tib * (omegadot(i, :)' - Tbi_dot * rpy_dot(i, :)'))';
%     end
    
    % Input torques
    Tau = (I * omegadot')' + cross(omega, (I * omega')', 2);

    system_state = [rvec(0), rpy, phiL, thetaL, ...
                        rvec(1), omega, phiLdot, thetaLdot, ...
                        rvec(2), omegadot, phiLddot, thetaLddot];

    control_input = [u1, Tau];

end
