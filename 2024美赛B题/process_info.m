function info = process_info(info)
    if info.time_steps > 1
        info.tspan = linspace(0, info.t_max, info.time_steps);
    else
        info.tspan = [0, info.t_max];
    end
    [info.x0, info.y0] = lonLat2Mercator(info.lon_0, info.lat_0);
    info.init_val = [[info.x0, info.y0, info.z0], info.v0];
end