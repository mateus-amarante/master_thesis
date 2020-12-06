function sample_fun = sinusoidal_trajectory(omega, magnitude, offset, n_deriv_out)

omega = omega(:)';
magnitude = magnitude(:)';
offset = offset(:)';

signal = [1 1 -1 -1];

sample_deriv = @(t, deriv) signal(rem(deriv,4) + 1)*magnitude.*omega.^deriv.*(rem(deriv,2)*cos(omega.*t) + ...
    ~rem(deriv,2)*(sin(omega.*t) + ~deriv*offset));

sample_fun = @(t) sample(t, length(omega), n_deriv_out);

    function v = sample(t, nvar, n_deriv_out) 
        v = zeros(length(t), nvar*(n_deriv_out+1));
        for i=0:n_deriv_out
            v(:,i*nvar+1:(i+1)*nvar) = sample_deriv(t,i);
        end
    end
end

