function [flat_outputs] = differentially_flat_trajectory(flat_rL, flat_yaw, physics_p)

    % Physics parameters renaming
    M = physics_p.M;
    g = physics_p.g;
    I = physics_p.I;
    Ix = physics_p.Ix;
    Iy = physics_p.Iy;
    Iz = physics_p.Iz;

    L = physics_p.L;
    m = physics_p.m;

    % Auxiliar variables
    ez = [0, 0, 1]';

    % Auxiliar functions
    rLvec = @(der)flat_rL(:, der * 3 + 1:(der + 1) * 3);
    yaw = @(der)flat_yaw(:, der);

    % State renaming
    % xLddot = rLvec(2)(:, 1);
    % yLddot = rLvec(2)(:, 2);
    % zLddot = rLvec(2)(:, 3);

    % xL3dot = rLvec(3)(:, 1);
    % yL3dot = rLvec(3)(:, 2);
    % zL3dot = rLvec(3)(:, 3);

    % xL4dot = rLvec(4)(:, 1);
    % yL4dot = rLvec(4)(:, 2);
    % zL4dot = rLvec(4)(:, 3);

    % Cable tension determination
    Tp = -m * rLvec(2) - m * g * ez';
    p = Tp ./ vecnorm(Tp, 2, 2);
    T = dot(Tp, p, 2);

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
    pdot = -(m * rLvec(3) + p .* Tdot) ./ T;

    aux2 = m * rLvec(4) + p .* Tddot + 2 * Tdot .* pdot;
    pddot = -aux2 ./ T;

    p3dot = (aux2 .* Tdot - (m * rLvec(5) + p .* T3dot + 2 * Tdot .* pddot + 3 * Tddot .* pdot) .* T) ./ T.^2;

    p4dot = (-2 * aux2 .* Tdot.^2 + ((m * rLvec(4) + p .* Tddot + Tdot .* pdot) .* Tddot + ...
        2 * (m * rLvec(5) + p .* T3dot + Tdot .* pddot + 2 * Tddot .* pdot) .* Tdot + ...
        2 * Tdot.^2 .* pddot + 3 * Tdot .* Tddot .* pdot) .* T - ...
        (m * rLvec(6) + p .* T4dot + 2 * Tdot .* p3dot + 5 * Tddot .* pddot + 4 * T3dot .* pdot) .* T.^2) ./ T.^3;

    % Drone state
    r = rLvec(0) - L * p;
    rdot = rLvec(1) - L * pdot;
    rddot = rLvec(2) - L * pddot;
    r3dot = rLvec(3) - L * p3dot;
    r4dot = rLvec(4) - L * p4dot;

    flat_outputs = [r rdot rddot r3dot r4dot];
    
end
