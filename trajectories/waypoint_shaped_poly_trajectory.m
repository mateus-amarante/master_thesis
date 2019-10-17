function sample_fun = waypoint_shaped_poly_trajectory(td, qd, n_deriv_out, physics_p)

wn = sqrt((physics_p.M+physics_p.m)*physics_p.g/(physics_p.M*physics_p.l));
zeta = 0;

poly_fun = plan_polynomial_trajectory2(td, [qd, repmat(zeros(size(qd)), 1, n_deriv_out)], size(qd,2), n_deriv_out);

sample_fun = @(t) zvd_shaper(t, poly_fun(t), wn, zeta);

end

