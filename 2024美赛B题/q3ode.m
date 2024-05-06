function dydt = q3ode(t, xyz_v, info)
    global rho_data;
    global current_data;
    % xyz_v长度为6，1~3是坐标，4~6是速度
    % J：旋转惯量
    dydt = zeros(6, 1);
    dydt(1) = xyz_v(4);
    dydt(2) = xyz_v(5);
    dydt(3) = xyz_v(6);
    
    tot_m = info.m + info.mw;
    % 获取当前时刻洋流，并计算水阻尼F_water
    [ux, uy] = current_data.F(xyz_v(1), xyz_v(2), xyz_v(3));
    rho = rho_data.F(xyz_v(1), xyz_v(2), xyz_v(3));
    const_Da = 0.5 * info.Cd * rho * pi * info.R^2;
    vr = [(xyz_v(4) - ux), (xyz_v(5) - uy), (xyz_v(6) - 0)]';
    F_water = abs(const_Da .* vr) .* vr;
    % 浮力G_eta
    G_eta = zeros(3, 1);
    G_eta(3) = rho * info.g * pi * info.R^3 * 4 / 3 - tot_m * info.g;

    % 速度对时间求导
    dydt(4:6) = (info.F(t) + info.f_other(t) - F_water + G_eta) ./ info.J;

end