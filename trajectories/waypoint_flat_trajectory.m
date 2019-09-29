function sample_fun = waypoint_flat_trajectory(td, rL_pos, yaw_pos, physics_p)

rL_fun = waypoint_poly_trajectory(td, rL_pos, 6);
yaw_fun = waypoint_poly_trajectory(td, yaw_pos, 2);

sample_fun = @(t) differentially_flat_trajectory(rL_fun(t), yaw_fun(t), physics_p);

end

