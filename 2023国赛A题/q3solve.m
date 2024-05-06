function obj = q3solve(vars)
sizeLimit = 350;
l = vars(1:4);        % 镜高
w = vars(5:8);        % 镜宽
h = vars(9:12);        % 镜中心高度
a = vars(13);
b = vars(14);
c = vars(15);
d = vars(16);
A_rlim = vars(17:19);

% l = 8 * ones(1, 4);
% w = 8 * ones(1, 4);
% h = vars(1:4);        % 镜中心高度
% a = vars(5);
% b = vars(6);
% c = vars(7);
% d = vars(8);
% A_rlim = vars(9:11);


A_sf = 1.9;

mirrors = [];

cur_R = 102.5;
for i = 1:4
    % 每一个区域的基本信息
    DM = sqrt(l(i).^2 + w(i).^2) + 5;
    Az = A_sf * DM;
    if i ~= 4
        max_Az = A_rlim(i) * Az;
    else
        max_Az = inf;
    end
    
    % 当前区域的方位角差环间距
    delta_R = sqrt(DM^2 - (Az / 2)^2);
    % 当前区域的方位角差
    d_alpha = asin(Az / (2 * cur_R));
    % 当前区域布置定日镜的个数
    N = round(2 * pi / d_alpha);
    % 当前区域第一个环为基本环
    flag = 1;
    % 退出信号
    exit_flag = 0;
    while true
        if flag == 1
            polar_angle = linspace(0, 0 + d_alpha * (N - 1.5), N)';
            mirrors = [mirrors; ...
                [cur_R .* cos(polar_angle), cur_R .* sin(polar_angle), ...
                repmat(h(i), N, 1), repmat(l(i), N, 1), repmat(w(i), N, 1)]];
        else
            polar_angle = linspace(d_alpha / 2, d_alpha / 2 + d_alpha * (N - 1.5), N)';
            mirrors = [mirrors; ...
                [cur_R .* cos(polar_angle), cur_R .* sin(polar_angle), ...
                repmat(h(i), N, 1), repmat(l(i), N, 1), repmat(w(i), N, 1)]];
        end

        % 判断是否到达镜场边缘
        if cur_R + delta_R > sizeLimit
            exit_flag = 1;
            break;
        end

        % 判断下一个半径是否触发区域转换
        tmp_R = cur_R + delta_R;
        tmp_Az = 2 * tmp_R * sin(d_alpha / 2);
        if tmp_Az > max_Az
            % 进行区域转换操作
            cur_R = cur_R + 8;
            % 直接退出while循环，计算下一个区域的信息
            break;
        else
            % 正常地进行半径增加操作
            cur_R = cur_R + delta_R;
            % 环类型切换
            flag = flag * -1;
        end

    end
    if exit_flag == 1
        % 提前退出指令
        break;
    end
end

% 使用两个二次函数进行分割
idx1 = a .* mirrors(:, 1).^2 + b <= mirrors(:, 2);
idx2 = c .* mirrors(:, 1).^2 + d >= mirrors(:, 2);
mirrors = mirrors(idx1 & idx2, :);

% points的各列信息严格对照问题一
DD = 1;
if DD == 1
    scatter(mirrors(:, 1), mirrors(:, 2), 5);
    axis equal;
    hold on;
end

%% 改写问题一的函数进行求解

% 相关参数设定

% 12个月，5个时间点
day_number = 12;
times_number = 5;
q3_days = datetime(2023, 1:day_number, 21);
% 一天5个时间点
q3_daytimes = [9, 10.5, 12, 13.5, 15];
% 春分日期
SpringEquinox = datetime(2023, 3, 21);

% --- 接收塔参数 ---
param.Hz = 80;

% 求解
P_year = 0;
Ps_year = 0;
for i = 1:day_number
    P_day = 0;
    Ps_day = 0;
    for j = 1:times_number
        ST = q3_daytimes(j);
        D = days(q3_days(i) - SpringEquinox);
%         disp(q2_days(i));
%         disp(ST);
        % 求此刻的DNI、太阳的高度角、方位角
        param = calDNI(D, ST , param);
        
        % 反射光线计算
        r = [0 - mirrors(:, 1), 0 - mirrors(:, 2), param.Hz - mirrors(:, 3)];
        dHR = sqrt(r(:, 1).^2 + r(:, 2).^2 + r(:, 3).^2);
        r = r ./ dHR;
        a_theta2 = sqrt((1 + param.light_vec * r') ./ 2)';
        n = (param.light_vec + r) ./ (2 .* a_theta2);  
        
        % 余弦效率
        mirrors(:, 6) = n * param.light_vec';
        
        % 大气效率
        mirrors(:, 7) = 0.99321 - 0.0001176 .* dHR + 1.97 .* 1e-8 .* dHR.^2;

        % 光学效率
        mirrors(:, 8) = 0.9 * 0.99 * 0.92 .* mirrors(:, 6) .* mirrors(:, 7);
        
        % 计算瞬时功率
        P_time = param.DNI .* sum(mirrors(:, 4) .* mirrors(:, 5) .* mirrors(:, 8));
        
        % 单位面积功率
        Ps_time = P_time / sum(mirrors(:, 4) .* mirrors(:, 5));

        % 累加操作
        P_day = P_day + P_time;
        Ps_day = Ps_day + Ps_time;
    end
    P_day = P_day / times_number;
    Ps_day = Ps_day / times_number;

    P_year = P_year + P_day;
    Ps_year = Ps_year + Ps_day;
end
P_year = P_year / day_number;
Ps_year = Ps_year / day_number;
if P_year < 60 * 1e3
    punish = abs(60 * 1e3 - P_year);
else
    punish = 0;
end

obj = 1 / Ps_year + punish;

end
