function eta_trunc = q2_cal_eta_trunc(mir_info, param)

sig_sun = 2.51;
sig_bq = 1.88;
sig_ast = 0;
sig_track = 0.63;

% 反射距离d计算
d = sqrt((mir_info(1)).^2 + (mir_info(2)).^2 + (mir_info(3) - param.Hz).^2);
sig_tot = sqrt(d^2 * (sig_sun^2 + sig_bq^2 + sig_ast^2 + sig_track^2)) * 1e-3;

% 使用简单椭圆积分进行估算

% 求矩形反射面板的等效半径R
R = sqrt(mir_info(4) * mir_info(5) / pi);

% 求吸热塔对定日镜的高度视角h_t
h_t = atan((param.Hz - mir_info(3)) / mir_info(6));

% 使用四象限正切得到方位角的关系
alpha_r = atan2(mir_info(10), mir_info(9));
alpha_t = atan2(-mir_info(2), -mir_info(1));

% 椭圆的a, b
a = R / cos(h_t);
b = abs(R * cos(alpha_r - alpha_t) * cos(mir_info(7) - h_t) / cos(h_t));

% 对5种情况进行积分
half_d = param.col_d / 2;
half_h = param.col_h / 2;
lower_fcn = @(x) -b .* sqrt(1 - (x ./ a).^2);
upper_fcn = @(x) b .* sqrt(1 - (x ./ a).^2);
z = @(x, y) exp(-(x.^2 + y.^2) ./ (2 .* sig_tot.^2));
if a < half_d && b < half_h
    % 椭圆在方框内
    x = [-a, a];
    eta_trunc = integral2(z, x(1), x(2), lower_fcn, upper_fcn) / (2 * pi * sig_tot^2);

elseif a > half_d && b < half_h
    % 长轴超过
    x = [-half_d, half_d];
    eta_trunc = integral2(z, x(1), x(2), lower_fcn, upper_fcn) / (2 * pi * sig_tot^2);

elseif a < half_d && b > half_h
    % 短轴超过
    x1 = [-a, -a * sqrt(1 - (half_h / b)^2)];
    x2 = [-a * sqrt(1 - (half_h / b)^2), a * sqrt(1 - (half_h / b)^2)];
    x3 = [a * sqrt(1 - (half_h / b)^2), a];
    I1 = integral2(z, x1(1), x1(2), lower_fcn, upper_fcn);
    I2 = integral2(z, x2(1), x2(2), -half_h, half_h);
    I3 = integral2(z, x3(1), x3(2), lower_fcn, upper_fcn);
    eta_trunc = (I1 + I2 + I3) / (2 * pi * sig_tot^2);
else
    % 长短轴均超过
    if half_d^2 / a^2 + half_h^2 / b^2 < 1
        % 方框完全在椭圆内部
        x = [-half_d, half_d];
        eta_trunc = integral2(z, x(1), x(2), -half_h, half_h) / (2 * pi * sig_tot^2);
    else
        % 方框有一部分在椭圆外部
        x1 = [-half_d, -a * sqrt(1 - (half_h / b)^2)];
        x2 = [-a * sqrt(1 - (half_h / b)^2), a * sqrt(1 - (half_h / b)^2)];
        x3 = [a * sqrt(1 - (half_h / b)^2), half_d];
        I1 = integral2(z, x1(1), x1(2), lower_fcn, upper_fcn);
        I2 = integral2(z, x2(1), x2(2), -half_h, half_h);
        I3 = integral2(z, x3(1), x3(2), lower_fcn, upper_fcn);
        eta_trunc = (I1 + I2 + I3) / (2 * pi * sig_tot^2);
    end
end

end