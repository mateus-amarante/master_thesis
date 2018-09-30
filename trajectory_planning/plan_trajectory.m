function coef = plan_trajectory(tspan,q0,qf)
% Plan linear/cubic/quintic trajectories for 'ncols' variables

% tspan = [t0 tf]
% q0 = [pos [vel0] [acc0]]' por coluna # Initial Point
% qf = [pos [velf] [accf]]' por coluna # End Point
% Velocities and accelerations are optional

% coef = [a0 a1 a2 a3 a4 a5]' por coluna # a0 + a1*t + a2*t² + a3*t³ + a4*t? + a5*t?

sizeq0 = size(q0);

if length(tspan)~=2 || tspan(1)>=tspan(2) || any(sizeq0~=size(qf)) || sizeq0(1)>3
    return;
end

t0 = tspan(1);
tf = tspan(2);

M = zeros(sizeq0(1)*2);

% Linear
if sizeq0(1) == 1
    
    M = [1 t0; 1 tf];
    
    % Cubic
elseif sizeq0(1) == 2
    
    M = [1 t0   t0^2   t0^3;
        0 1  2*t0   3*t0^2;
        1 tf   tf^2   tf^3;
        0 1  2*tf   3*tf^2];
    
    % Quintic
else % sizeq0(1) == 3
    
    M = [1 t0   t0^2   t0^3    t0^4    t0^5;
        0 1  2*t0   3*t0^2  4*t0^3  5*t0^4;
        0 0  2      6*t0   12*t0^2 20*t0^3;
        1 tf   tf^2   tf^3    tf^4    tf^5;
        0 1  2*tf   3*tf^2  4*tf^3  5*tf^4;
        0 0  2      6*tf   12*tf^2 20*tf^3];
    
end

coef = zeros(6,sizeq0(2));

coef(1:sizeq0(1)*2,:) = M\[q0;qf];
%     coef = M\[q0;qf];

end
