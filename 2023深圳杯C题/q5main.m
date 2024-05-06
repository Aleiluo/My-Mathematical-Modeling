%% 问题五求解
xb_range = 1000:20:10000;
vb_range = 10:2:50;
xb_len = length(xb_range);
vb_len = length(vb_range);
Aarrive_time = zeros(xb_len, vb_len);
Barrive_time = zeros(xb_len, vb_len);

for i = 1:xb_len
    for j = 1:vb_len
        xb = xb_range(i);
        vb = vb_range(j);
        fprintf("B点坐标：%d, 无人机B速度：%f\n", xb, vb);
        map = getQ5map(xb);
        [droneA, droneB, dwaParam] = get_param(vb, map, false);
        % 使用问题二的求解代码即可
        res = q3_q5DWAsolve(droneA, droneB, dwaParam, map);
        Aarrive_time(i, j) = res.A_time;
        Barrive_time(i, j) = res.B_time;
    end
end

%% 绘制曲面图
close all;
load q5res.mat
xb_range = 1000:20:10000;
vb_range = 10:2:50;
draw_surf(smoothdata(Aarrive_time), xb_range, vb_range);
draw_surf(smoothdata(Barrive_time), xb_range, vb_range);
%% 生成4象限轨迹图

% % 近慢
% vb = 15;
% xb = 1000;

% % 近快
% vb = 50;
% xb = 1000;
% 
% 远慢
% vb = 15;
% xb = 5000;
% 
% 远快
vb = 50;
xb = 7000;

map = getQ5map(xb);
[droneA, droneB, dwaParam] = get_param(vb, map, true);
dwaParam.drawTangent = false;
dwaParam.gif = false;
dwaParam.gif_dt = 0.03;
dwaParam.gif_name = 'q5远快动图.gif';
q3_q5DWAsolve(droneA, droneB, dwaParam, map);
print("q5远快v50x7000.png", '-dpng','-r600');

%% 地图信息函数
function map = getQ5map(xb)
    % 求切点即可
    map.R = 500;
    map.A = [-1000, 0];
    map.B = [xb, 0];
    kB = sqrt(map.R^2 / (map.B(1)^2 - map.R^2));
    kA = sqrt(map.R^2 / (map.A(1)^2 - map.R^2));
    map.thetaB = abs(atan(kB));
    map.thetaA = abs(atan(kA));
    map.cutC1 = [-map.R * sin(map.thetaA), map.R * cos(map.thetaA)];
    map.cutC2 = [map.R * sin(map.thetaB), map.R * cos(map.thetaB)];
    map.cutC3 = [-map.R * sin(map.thetaA), -map.R * cos(map.thetaA)];
    map.cutC4 = [map.R * sin(map.thetaB), -map.R * cos(map.thetaB)];
end

function [droneA, droneB, dwaParam] = get_param(vb, map, Display)
    t_s = @(x) ((12 - 7) / (10 - 50)) * (x - 10) + 12;

    % 定义无人机A参数
    droneA.name = 'A';
    droneA.curpos = [map.A(1), map.A(2), 0, 10, 0];   % [x, y, theta, v, w]
    droneA.theta_g = -pi / 2;
    droneA.goal = [map.cutC3(1), map.cutC3(2); ...
                   map.cutC4(1), map.cutC4(2); ...
                   map.B(1), map.B(2)];
    droneA.curgoal = 1;                      % 无人机目标(行)
    droneA.v = 10;                           % 无人机速度
    droneA.a_max = 0;                        % 无人机加速度
    droneA.w_max = droneA.v / 30;            % 无人机最大角速度
    droneA.alpha_max = 100 * pi /180;        % 无人机最大角加速度
    droneA.headingWeight = 0.5;             % 终点朝向权重
    droneA.dis2goalWeight = 2;            % 终点距离权重
    droneA.StaticDisWeight = 4;           % A逃离静态障碍的意愿
    droneA.DynamicDisWeight = 2;          % A逃离动态障碍的意愿
    droneA.minDis2StaticObstacle = 5 * droneA.v;     % A侦测静态障碍的距离
    droneA.minDis2DynamicObstacle = 5 * droneA.v;    % A侦测动态障碍的距离
    droneA.sampleDuration = 13;            % 采样时长
    
    % 定义无人机B参数
    droneB.name = 'B';
    droneB.curpos = [map.B(1), map.B(2), pi, vb, 0];
    droneB.theta_g = pi / 2;
    droneB.goal = [map.cutC2(1), map.cutC2(2); ...
                   map.cutC1(1), map.cutC1(2); ...
                   map.A(1), map.A(2)];
    droneB.curgoal = 1;
    droneB.v = vb;
    droneB.a_max = 0;
    droneB.w_max = droneB.v / 30;        
    droneB.alpha_max = 100 * pi /180;
    droneB.headingWeight = 0.5; 
    droneB.dis2goalWeight = 2; 
    droneB.StaticDisWeight = 4;
    droneB.DynamicDisWeight = 2;
    droneB.minDis2StaticObstacle = 5 * droneB.v;
    droneB.minDis2DynamicObstacle = 5 * droneB.v; 
    droneB.sampleDuration = t_s(droneB.v);

    % 求解器参数
    % 注意：限制最大迭代次数很重要，能大幅提高并行求解效率
    dwaParam.dt = 0.3;                     % 时间分割间隔
    dwaParam.dw = 0.1 * pi /180;             % 角度分辨率
    dwaParam.maxIter = round(2000 / dwaParam.dt);
    if Display == true
        dwaParam.displayIter = 10;
    else
        dwaParam.displayIter = inf;
    end
    dwaParam.drawTangent = true;
end

function [] = draw_surf(mat, xb_range, vb_range)
    % 绘制曲面图
    figure;

    % 创建一个网格
    [X, Y] = meshgrid(vb_range, xb_range);

    % 绘制曲面图
    surf(X, Y, mat, 'EdgeColor', 'none', 'FaceColor', 'interp', 'FaceAlpha', 0.85);

    % 添加标题和标签
    xlabel('V_b','FontSize',13);
    ylabel('X_b','FontSize',13);
    zlabel('总耗时','FontSize',13);

    % 调整图形显示
    axis tight;
    grid on;

    % 添加颜色映射
    color_values = [
        3,169,244;
        102,187,106;
        255,152,0;
        233,30,99;
        244,67,54;
    ] / 255;
    colormap(GenColormap(color_values, 64));

    % 添加色彩栏
    colorbar;

end

function colormap = GenColormap(map, n)
    m = size(map, 1);
    if m >= n
        colormap = map;
    else
        % 范围重置
        range = 0 : m-1;
        range = range*(n-1)/(m-1) + 1;
        % 插值
        colormap = nan(n, 3);
        for i = 1:3
            colormap(:, i) = interp1(range, map(:, i), 1:n);
        end
    end
end






