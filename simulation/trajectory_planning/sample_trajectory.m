function qd = sample_trajectory(coef,t)
    % coef = [a0 a1 a2 a3 a4 a5]' per var (column) for 'n' variables
    % a0 + a1*t + a2*t² + a3*t³ + a4*t? + a5*t?
    % qd =[pos1 pos2 ... posn,
    %      vel1 vel2 ... veln,
    %      vel1 vel2 ... veln]' per time (column) for 'n' variables

    nvar = size(coef,2);
    
    qd = zeros(nvar*3,length(t));  

    t1 = t(:)';
    t0 = ones(size(t1));
    t2 = t1.^2;
    t3 = t1.^3;
    t4 = t1.^4;
    t5 = t1.^5;
    zero = zeros(size(t1));
    
    t_pos = [t0;      t1;   t2;   t3;    t4;    t5];
    t_vel = [ zero;   t0; 2*t1; 3*t2;  4*t3;  5*t4];
    t_acc = [ zero; zero; 2*t0; 6*t1; 12*t2; 20*t3];
    
    qd(1:nvar,:) = coef'*t_pos; % position
    qd(nvar+1:2*nvar,:) = coef'*t_vel; % velocity
    qd(2*nvar+1:end,:) = coef'*t_acc; % acceleration
    
end
