function [flat_outputs] = differentially_flat_trajectory(flat_rL, flat_yaw, physics_p)

    % Physics parameters renaming
    M = physics_p.M;
    g = physics_p.g;
    I = physics_p.I;

    l = physics_p.l;
    m = physics_p.m;

    Cd = physics_p.Cd(:)'; % drag coefficients of the aircraft
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
 
    T = vecnorm(Tp(0), 2, 2);
    pvec = Tp(0) ./ T;
    
    Tdot = dot(Tp(1), pvec, 2);
    pvec_dot = (Tp(1) - Tdot.*pvec) ./ T;
    
    Tddot = dot(Tp(2), pvec, 2) + dot(Tp(1), pvec_dot, 2);
    pvec_ddot = (Tp(2) - Tddot.*pvec - 2*Tdot.*pvec_dot) ./ T;
    
    T3dot = dot(Tp(3), pvec, 2) + 2*dot(Tp(2), pvec_dot, 2) + dot(Tp(1), pvec_ddot, 2);
    pvec_3dot = (Tp(3) - T3dot.*pvec - 3*Tddot.*pvec_dot - 3*Tdot.*pvec_ddot) ./ T;
    
    T4dot = dot(Tp(4), pvec, 2) + 3*dot(Tp(3), pvec_dot, 2) + 3*dot(Tp(2), pvec_ddot, 2) + dot(Tp(1), pvec_3dot, 2); 
    pvec_4dot = (Tp(4) - T4dot.*pvec - 4*T3dot.*pvec_dot - 6*Tddot.*pvec_ddot - 4*Tdot.*pvec_3dot) ./ T;  
    
    pvec_mat = [pvec, pvec_dot, pvec_ddot, pvec_3dot, pvec_4dot];
    p = @(der) pvec_mat(:, der * 3 + 1:(der + 1) * 3);
    
%     T_mat = [T Tdot Tddot T3dot T4dot];
%     T = @(der) T_mat(:, der + 1);
    
    % Drone state
    rvec = @(der) rLvec(der) - l*p(der);
    
    % Compute load angle
    thetaL = asin(-pvec(:, 1));
    phiL = asin(pvec(:, 2) ./ cos(thetaL));
%     phiL = -acos(pvec(:, 3) ./ cos(thetaL));

    % Compute load angular velocity/acceleration
    thetaLdot = -pvec_dot(:, 1) ./ cos(thetaL);
    phiLdot = (sin(phiL) .* sin(thetaL) .* thetaLdot + pvec_dot(:, 2)) ./ (cos(phiL) .* cos(thetaL));

    thetaLddot = (sin(thetaL) .* thetaLdot.^2 - pvec_ddot(:, 1)) ./ cos(thetaL);
    phiLddot = (sin(phiL) .* sin(thetaL) .* thetaLddot + 2 * cos(phiL) .* sin(thetaL) .* phiLdot .* thetaLdot + ...
        sin(phiL) .* cos(thetaL) .* (phiLdot.^2 + thetaL.^2) + pvec_ddot(:, 2)) ./ (cos(phiL) .* cos(thetaL));
    
    % Rotation Matrix
    u1vec = M * rvec(2) - Tp(0) + M * g * ez + Cd.*rvec(1);
    % Thrust force
    u1 = vecnorm(u1vec, 2, 2);
    
    ezb = u1vec ./ u1;

%     exc = [cos(yaw(0)), sin(yaw(0)), zeros(size(u1))];
%     
%     aux = cross(ezb, exc, 2);
%     eyb = aux ./ vecnorm(aux, 2, 2);
%     exb = cross(eyb, ezb, 2);
    eyc = [-sin(yaw(0)), cos(yaw(0)), zeros(size(u1))];
    
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
    psi = rpy(:, 3);

    % Angular velocity
    aux = M*rvec(3) + Cd.*rvec(2) - Tp(1);
    u1dot = dot(aux, ezb, 2);
    hw = (aux - u1dot .* ezb) ./ u1;

    pp = -dot(hw, eyb, 2);
    q = dot(hw, exb, 2);
    r = (cos(theta).*yaw(1) - sin(phi).*q)./cos(phi);

    omega = [pp, q, r];
    rpy_dot = zeros(size(ezb));
    for i = 1:size(Rvec, 1)
        Tbi = Tb2i(phi(i), theta(i));
        rpy_dot(i, :) = (Tbi * omega(i, :)')';
    end
    phidot = rpy_dot(:, 1);
    thetadot = rpy_dot(:, 2);
    psidot = rpy_dot(:, 3);

    % Angular acceleration
    aux2 = M*rvec(4) + Cd.*rvec(3) - Tp(2) - cross(omega, cross(omega, u1vec, 2), 2);
    u1ddot = dot(aux2, ezb, 2);
    halpha = (aux2 - u1ddot .* ezb - 2 * cross(omega, u1dot .* ezb, 2))./u1;

    pdot = -dot(halpha, eyb, 2);
    qdot = dot(halpha, exb, 2);
    rdot = (cos(theta).*yaw(2) - sin(phi).*qdot - thetadot.*(phidot + sin(theta).*yaw(1)))./cos(phi);

    omegadot = [pdot, qdot, rdot];

    rpy_ddot = zeros(size(ezb));
    for i = 1:size(Rvec, 1)
        
        Tbi = Tb2i(rpy(i, 1), rpy(i, 2));
        Tbi_dot = Tb2i_dot(rpy(i, 1), rpy(i, 2), rpy_dot(i, 1), rpy_dot(i, 2));
        rpy_ddot(i, :) = (Tbi_dot * omega(i, :)' + Tbi * omegadot(i, :)')';
%         rpy_ddot(i,1) = (sin(phi(i)).*qdot(i) + cos(phi(i)).*rdot(i) + phidot(i).*thetadot(i)).*tan(theta(i)) + pdot(i) + psidot(i).*thetadot(i)./cos(theta(i));
%         rpy_ddot(i,2) = -sin(phi(i)).*rdot(i) + cos(phi(i)).*qdot(i) - phidot(i).*(q(i).*sin(phi(i)) + r(i).*cos(phi(i)));
    end
    
%     rpy_ddot(:, 3) = yaw(2);
    
    phiddot = rpy_ddot(:, 1);
    thetaddot = rpy_ddot(:, 2);
    psiddot = rpy_ddot(:, 3);
    
    % Input torques
    Tau = (I * omegadot')' + cross(omega, (I * omega')', 2);

    system_state = [rvec(0), rpy, phiL, thetaL, ...
                    rvec(1), rpy_dot, phiLdot, thetaLdot, ...
                    rvec(2), rpy_ddot, phiLddot, thetaLddot];

    control_input = [u1, Tau];

    flat_outputs = [system_state, control_input];
end
