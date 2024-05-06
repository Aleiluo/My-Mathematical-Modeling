function loss = v_loss_Fb(Fb, std_v, info)
    info.F(3) = Fb;
    DriftOde = @(t, x) q1ode(t, x, info);
    options = odeset('Events', @(t,xyz_v) checkOdeBounds(t, xyz_v));
    [t, xyz_v] = ode45(DriftOde, info.tspan, info.init_val, options);

    if size(xyz_v, 1) > 30
        loss = sum((xyz_v(end-30:end, 6) - std_v).^2);
    else
        diff_v = abs(xyz_v(:, 6) - std_v); % 计算每个速度与std_v的差值
        [~, idx] = min(diff_v); % 找到最接近std_v的速度的索引
        start_idx = min(idx, length(xyz_v)); 
        loss = sum((xyz_v(start_idx:end, 6) - std_v).^2);
    end

end
