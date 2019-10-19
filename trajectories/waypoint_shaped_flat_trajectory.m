function sample_fun = waypoint_shaped_flat_trajectory(td, rL_pos, yaw_pos, physics_p)

rL_fun = waypoint_shaped_poly_trajectory(td, rL_pos, 6, physics_p, .001);
yaw_fun = waypoint_shaped_poly_trajectory(td, yaw_pos, 2, physics_p, .001);

sample_fun = @(t) differentially_flat_trajectory(rL_fun(t), yaw_fun(t), physics_p);

end

