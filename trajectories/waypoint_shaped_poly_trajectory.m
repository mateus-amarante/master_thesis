function sample_fun = waypoint_shaped_poly_trajectory(td, qd, n_deriv_out, physics_p, dt)

poly_fun = waypoint_poly_trajectory(td, qd, n_deriv_out);

tt = td(1):dt:td(end);
[qqd,ts,A] = zvd_shaper(tt, poly_fun(tt), physics_p.wn, physics_p.zeta);

sample_fun = @(t) interp1(tt,qqd,t);

end