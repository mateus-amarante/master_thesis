function [flat_outputs] = differentially_flat_trajectory(flat_rL, flat_yaw, physics_p)

    % Physics parameters renaming
    M = physics_p.M;
    g = physics_p.g;
    I = physics_p.I;

    L = physics_p.L;
    m = physics_p.m;

    % Auxiliar variables
    ez = [0, 0, 1];

    % Auxiliar functions
    rLvec = @(der)flat_rL(:, der * 3 + 1:(der + 1) * 3);
    yaw = @(der)flat_yaw(:, der + 1);

    % Cable tension determination
    Tp = -m * rLvec(2) - m * g * ez;
    pvec = Tp ./ vecnorm(Tp, 2, 2);
    T = dot(Tp, pvec, 2);

    % Tension magnitude derivatives
    Tdot = m^2 * dot(rLvec(2) + [0, 0, g], rLvec(3), 2) ./ T;

    aux4 = m^2 * (dot(rLvec(2) + [0, 0, g], rLvec(4), 2) + dot(rLvec(3), rLvec(3), 2));
    Tddot = (aux4 - Tdot.^2) ./ T;

    aux5 = m^2 * (dot(rLvec(2) + [0, 0, g], rLvec(5), 2) + 3 * dot(rLvec(3), rLvec(4), 2));
    T3dot = ((-aux4 + Tdot.^2) .* Tdot + (aux5 - 2 * Tdot .* Tddot) .* T) ./ T.^2;

    aux6 = m^2 * (dot(rLvec(2) + [0, 0, g], rLvec(6), 2) + 4 * dot(rLvec(3), rLvec(5), 2) + 3 * dot(rLvec(4), rLvec(4), 2));
    T4dot = (2 * (aux4 - Tdot.^2) .* Tdot.^2 + (-aux4 .* Tddot - 2 * aux5 .* Tdot + 5 * Tdot.^2 .* Tddot) .* T + ...
        (aux6 - 2 .* Tdot .* T3dot - 2 * Tddot.^2) .* T.^2) ./ T.^3;

    % Tension unit vector derivatives
    pvecdot = -(m * rLvec(3) + pvec .* Tdot) ./ T;

    aux2 = m * rLvec(4) + pvec .* Tddot + 2 * Tdot .* pvecdot;
    pddot = -aux2 ./ T;

    p3dot = (aux2 .* Tdot - (m * rLvec(5) + pvec .* T3dot + 2 * Tdot .* pddot + 3 * Tddot .* pvecdot) .* T) ./ T.^2;

    p4dot = (-2 * aux2 .* Tdot.^2 + ((m * rLvec(4) + pvec .* Tddot + Tdot .* pvecdot) .* Tddot + ...
        2 * (m * rLvec(5) + pvec .* T3dot + Tdot .* pddot + 2 * Tddot .* pvecdot) .* Tdot + ...
        2 * Tdot.^2 .* pddot + 3 * Tdot .* Tddot .* pvecdot) .* T - ...
        (m * rLvec(6) + pvec .* T4dot + 2 * Tdot .* p3dot + 5 * Tddot .* pddot + 4 * T3dot .* pvecdot) .* T.^2) ./ T.^3;

    % Compute load angle
    phiL = asin(pvec(:, 2));
    thetaL = asin(-pvec(:, 1) ./ cos(phiL));

    % Compute load angular velocity/acceleration
    phiLdot = pvecdot(:, 2) ./ cos(phiL);
    thetaLdot = (sin(phiL) .* sin(thetaL) .* phiLdot - pvecdot(:, 1)) ./ (cos(phiL) .* cos(thetaL));

    phiLddot = (pddot(:, 2) + sin(phiL) .* phiLdot.^2) ./ cos(phiL);
    thetaLddot = (sin(phiL) .* sin(thetaL) .* phiLddot + 2 * sin(phiL) .* cos(thetaL) .* phiLdot .* thetaLdot + ...
        cos(phiL) .* sin(thetaL) .* (phiLdot.^2 + thetaL.^2)) ./ (cos(phiL) .* cos(thetaL));

    % Drone state
    rvec = rLvec(0) - L * pvec;
    rvecdot = rLvec(1) - L * pvecdot;
    rddot = rLvec(2) - L * pddot;
    r3dot = rLvec(3) - L * p3dot;
    r4dot = rLvec(4) - L * p4dot;

    % Rotation Matrix
    t = M * rddot - Tp + M * g * ez;
    ezb = t ./ vecnorm(t, 2, 2);

    exc = [cos(yaw(0)), sin(yaw(0)), zeros(size(t, 1), 1)];

    aux = cross(ezb, exc, 2);
    eyb = aux ./ vecnorm(aux, 2, 2);
    exb = cross(eyb, ezb, 2);

    Rvec = [exb, eyb, ezb];

    % Thrust force
    u1 = vecnorm(t, 2, 2);

    % Angular velocity
    u1dot = M * dot(r3dot, ezb, 2);
    hw = M ./ u1 .* (r3dot - dot(r3dot, ezb, 2) .* ezb);

    p = -dot(hw, eyb, 2);
    q = dot(hw, exb, 2);
    r = dot(yaw(1) .* ez, ezb, 2);

    omega = [p, q, r];

    % Angular acceleration
    aux = cross(omega, cross(omega, u1 .* ezb, 2), 2);
    u1ddot = M * dot(r4dot, ezb, 2) - dot(aux, ezb, 2);
    halpha = M * r4dot - u1ddot .* ezb - 2 * cross(omega, u1dot .* ezb, 2) - aux;

    pdot = -dot(halpha, eyb, 2);
    qdot = dot(halpha, exb, 2);
    rdot = dot(yaw(2) .* ez, ezb, 2);

    omegadot = [pdot, qdot, rdot];

    % Input torques (TODO)
    Tau = (I * omegadot')' + cross(omega, (I * omega')', 2);

    % Euler angles and angular velocity/acceleration in inertial frame

    rpy = zeros(size(ezb));
    rpy_dot = rpy;
    rpy_ddot = rpy;

    for i = 1:size(Rvec, 1)
        rpy(i, :) = fliplr(rotm2eul(reshape(Rvec(i, :), [3, 3])));

        Tib = Ti2b(rpy(i, 1), rpy(i, 2));
        Tbi = Tb2i(rpy(i, 1), rpy(i, 2));
        Tbi_dot = Tb2i_dot(rpy(i, 1), rpy(i, 2), omega(i, 1), omega(i, 2));

        rpy_dot(i, :) = (Tbi * rpy(i, :)')';
        rpy_ddot(i, :) = (Tib * (omegadot(i, :)' - Tbi_dot * rpy_dot(i, :)'))';
    end

    flat_outputs = [rvec, rpy, phiL, thetaL, ...
                        rvecdot, omega, phiLdot, thetaLdot, ...
                        rddot, omegadot, phiLddot, thetaLddot];
    % flat_outputs = [rvec, rvedot, rddot, r3dot, r4dot, rpy, phiL, thetaL, pqr, rqpy_dot, phiLdot, thetaLdot];

end
