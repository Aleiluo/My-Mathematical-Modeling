clc,clear;
clear global;
lon_range = [14.6, 22];
lat_range = [34.96, 40.43];
init_data(lon_range, lat_range);
global depth_data;
global rho_data;
global current_data;

%% 测试使用

requested_longitude = 18.0;
requested_latitude = 38;
z = -10;
[x, y] = lonLat2Mercator(requested_longitude, requested_latitude);
get_rho(x, y, z)

%% 求解方程
info.R = 2.2854;
info.r = info.R - 70 * 10^-3; 
info.m = 20 * 10^3;
% 注水量
info.mw = 0;
info.J = 2 * info.m * ((info.R^5 - info.r^5) / (info.R^3 - info.r^3)) / 5;
info.Cd = 0.01;
info.g = 9.8;
info.F = [0, 0, 0]';
info.f_other = [0, 0, 0]';
info.lon_0 = 18;
info.lat_0 = 36;
info.z0 = -10;
info.v0 = [0, 0, 0];
info.t_max = 3 * 3600;
info.time_steps = 3 * 3600;
info = process_info(info);

std_v = -1;

% 求下潜需要额外注水的重量mw
mw = 3.194595461978635e+04;
% mw = cal_mw(std_v, info);

info.mw = mw;
info.t_max = 3 * 3600;
info.time_steps = 3 * 3600;
info = process_info(info);

DriftOde = @(t, x) q1ode(t, x, info);
options = odeset('Events', @(t,xyz_v) checkOdeBounds(t, xyz_v));
[t, xyz_v] = ode45(DriftOde, info.tspan, info.init_val, options);
%% 保存数据
writematrix(t, 'data2py/mw.xlsx',Sheet='t', WriteMode='overwritesheet');
writematrix(xyz_v, 'data2py/mw.xlsx',Sheet='xyz_v', WriteMode='overwritesheet');

%% 求临界力Fb
info.lon_0 = 18;
info.lat_0 = 36;
info.z0 = -1500;
info.v0 = [0.15, 0.2, -0.5];
info.t_max = 10 * 3600;
info.time_steps = 0;
info = process_info(info);
% v=0,Fb=72.226647958746090;
% v=1,Fb=1.320633053524231e+02;
std_v = 1;
Fb = cal_Fb(std_v, info);
% Fb = 138.326852;
info.F(3) = Fb;
info.t_max = 10 * 3600;
info.time_steps = 10 * 3600;
info = process_info(info);
DriftOde = @(t, x) q1ode(t, x, info);
options = odeset('Events', @(t,xyz_v) checkOdeBounds(t, xyz_v));
[t, xyz_v] = ode45(DriftOde, info.tspan, info.init_val, options);
%% 保存数据
writematrix(t, 'data2py/Fb.xlsx',Sheet='t1', WriteMode='overwritesheet');
writematrix(xyz_v, 'data2py/Fb.xlsx',Sheet='xyz_v1', WriteMode='overwritesheet');
%% 小于临界浮力进行演算
info.lon_0 = 18;
info.lat_0 = 36;
info.z0 = -1500;
info.v0 = [0.15, 0.2, -0.5];
info.F(3) = 20;
info.t_max = 10 * 3600;
info.time_steps = 10 * 3600;
info = process_info(info);

DriftOde = @(t, x) q1ode(t, x, info);
options = odeset('Events', @(t,xyz_v) checkOdeBounds(t, xyz_v));
[t, xyz_v] = ode45(DriftOde, info.tspan, info.init_val, options);
%% 
writematrix(t, 'data2py/sink.xlsx',Sheet='t', WriteMode='overwritesheet');
writematrix(xyz_v, 'data2py/sink.xlsx',Sheet='xyz_v', WriteMode='overwritesheet');
%% 卡尔曼增益

% 等于临界浮力进行演算
info.F(3) = 72.226647958746090;
info.mw = mw;
info.lon_0 = 18;
info.lat_0 = 36;
info.z0 = -1500;
info.v0 = [0.15, 0.2, -0.5];
info.t_max = 10 * 3600;
info.time_steps = 10 * 3600;
info = process_info(info);

DriftOde = @(t, x) q1ode(t, x, info);
options = odeset('Events', @(t,xyz_v) checkOdeBounds(t, xyz_v));
[t, xyz_v] = ode45(DriftOde, info.tspan, info.init_val, options);
%% 
writematrix(t, 'data2py/suspension.xlsx',Sheet='t', WriteMode='overwritesheet');
writematrix(xyz_v, 'data2py/suspension.xlsx',Sheet='xyz_v', WriteMode='overwritesheet');
%% 卡尔曼增益

% 损毁后总共发送信息次数
deliver_times = 5;
delta_t = info.t_max / deliver_times;
% 估计sigma_f
sigma_f = norm(info.F) * 0.1 / 3;
% 估计预测值sigma_1
sigma_1 = delta_t * sigma_f / info.J;
delta_index = round(info.time_steps / deliver_times);
received_t = t(round(delta_index:delta_index:deliver_times * delta_index));
received_data = xyz_v(round(delta_index:delta_index:deliver_times * delta_index), :);

v = received_data(:, 4:6);

% 通过加噪声的方式模拟接收到的数据
noise_level = 0.01;
noise = noise_level * randn(size(v));
p = v + noise;

s = zeros(size(p));

all_sigma_2 = [];

for i = 1:size(p, 1)
    % 一次一次地接收数据p，每次接收算一次sigma_2
    sigma_2 = norm(p(i, :)) * 0.1 / 3;
    all_sigma_2 = [all_sigma_2, sigma_2];
    s(i, :) =  (sigma_2^2 / (sigma_1^2 + sigma_2^2)) .* v(i, :) + (sigma_1^2 / (sigma_1^2 + sigma_2^2)) .* p(i, :);
end

%% 函数

function mw = cal_mw(std_v, info)
    % 给定下潜标准稳速，求无动力下潜的注水量
    mw_l = 31800;
    mw_r = 32000;
    tol = 1e-4;
    while abs(mw_r - mw_l) > tol
        mid1 = mw_l + (mw_r - mw_l) / 3;
        mid2 = mw_r - (mw_r - mw_l) / 3;
        % 如果注水量mid1的稳速比注水量mid2的稳速更接近std_v，将右边界定为mid2, 否则左边界定为mid1
        if v_loss_mw(mid1, std_v, info) < v_loss_mw(mid2, std_v, info)
            mw_r = mid2;
        else
            mw_l = mid1;
        end
    end
    mw = (mw_l + mw_r) / 2;
end

function Fb = cal_Fb(std_v, info)
    % 注水后求临界牵引力
    Fb_l = 0;
    Fb_r = 200;
    tol = 1e-4;
    while abs(Fb_r - Fb_l) > tol
        mid1 = Fb_l + (Fb_r - Fb_l) / 3;
        mid2 = Fb_r - (Fb_r - Fb_l) / 3;
        if v_loss_Fb(mid1, std_v, info) < v_loss_Fb(mid2, std_v, info)
            Fb_r = mid2;
        else
            Fb_l = mid1;
        end
    end
    Fb = (Fb_l + Fb_r) / 2;
end





