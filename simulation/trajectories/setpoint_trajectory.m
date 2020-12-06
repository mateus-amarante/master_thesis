function sample_fun = setpoint_trajectory(startpoint, endpoint, wait_time)

startpoint = startpoint(:);
endpoint = endpoint(:);

sample_fun = @(t) startpoint'.*(t < wait_time).*ones(length(t), length(startpoint)) + ...
    endpoint'.*(t >= wait_time).*ones(length(t), length(endpoint));

end

