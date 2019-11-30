function plot_phase_portrait(q,qdot,q_name,qdot_name,lambda, dict)

figure;
plot(q,qdot)

if nargin > 2
    if lambda ~= 0
        hold on;
        plot(q, -lambda*q);
        hold off;
    end
end

xlabel(q_name);
ylabel(qdot_name);

end

