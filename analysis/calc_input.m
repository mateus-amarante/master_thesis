function [u,s,ueq,usw] = calc_input(t,x,physics_p,traj_p,control_p)

% Compute desired trajectory
qd = traj_p.sample_fun(t);

u = zeros(length(t), control_p.n_inputs);
s = u;
ueq = u;
usw = u;

% TODO: define u for all timespecs at once
for i=1:length(t)
    [u(i,:), s(i,:), ueq(i,:), usw(i,:)] = control_p.control_fun(x(i,:),qd(i,:),physics_p,control_p);
end

end

