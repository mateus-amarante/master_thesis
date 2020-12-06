function sample_fun = waypoint_poly_trajectory(td, qd, n_deriv_out)

sample_fun = plan_polynomial_trajectory2(td, [qd, repmat(zeros(size(qd)), 1, n_deriv_out)], size(qd,2), n_deriv_out);

end

