%% 本程序使用DWA算法实现了双无人机带约束的避障
warning('off')
clc,clear;
close all;

%% 连续位置测试
% 修改建议：B站点到圆心的距离，只需考虑在[0.5,10] km内变化。
% 算得快，步长可以设置小一点
xb_range = 500:10:10000;
xb_len = length(xb_range);
Aarrive_time = zeros(1, xb_len);
Barrive_time = zeros(1, xb_len);
vb = 10;

for i = 1:xb_len
    xb = xb_range(i);
    fprintf("B点坐标：%d\n", xb);
    map = getQ3map(xb);
    [droneA, droneB, dwaParam] = get_param(vb, map, false);
    res = q3_q5DWAsolve(droneA, droneB, dwaParam, map);
    Aarrive_time(i) = res.A_time;
    Barrive_time(i) = res.B_time;
end

%% 做线性拟合工作
close all;
load q3res.mat

xb_range = 500:2:10000;
useA_idx = xb_range >= 2000 & xb_range <= 4000;
useB_idx = xb_range >= 2000 & xb_range <= 3000;

coeffA = polyfit(xb_range(useA_idx), Aarrive_time(useA_idx), 1);  % 对第一条直线进行线性拟合
coeffB = polyfit(xb_range(useB_idx), Barrive_time(useB_idx), 1);  % 对第二条直线进行线性拟合

x0 = (coeffB(2) - coeffA(2)) / (coeffA(1) - coeffB(1));

disp(['交点横坐标为: ', num2str(x0)]);

% 看看交点出无人机如何飞行的
% map = getQ3map(2050);
% [droneA, droneB, dwaParam] = get_param(2050, map, true);
% q2DWAsolve(droneA, droneB, dwaParam, map)

% 绘图
figure;
plot(xb_range, Aarrive_time, 'Color', '#F1AF7B','LineWidth',1.7);
hold on;
plot(xb_range, Barrive_time, 'Color', '#005B9A','LineWidth',1.7);
new_x = 1900:4000;
plot(new_x, polyval(coeffA, new_x), 'r--', 'LineWidth', 1.7);
plot(new_x, polyval(coeffB, new_x), 'g--', 'LineWidth', 1.7);
plot(x0, polyval(coeffA, x0), 'kx', 'MarkerSize', 10, 'LineWidth', 1.5);
text(x0 + 200, polyval(coeffA, x0) - 20, ['交点坐标: (' num2str(x0) ', ' num2str(polyval(coeffA, x0)) ')'], 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
xlabel('X_b');
ylabel('总耗时');
xlim([500,10000]);
legend('无人机A','无人机B','Location','northwest');
grid on;

print('q3无人机B位置-耗时曲线.png', '-dpng','-r600');

%% 绘出几种不同情况
% vb = 10;
% xb = 600;
% map = getQ3map(xb);
% [droneA, droneB, dwaParam] = get_param(vb, map, true);
% dwaParam.drawTangent = true;
% dwaParam.gif = true;
% dwaParam.gif_dt = 0.07;
% dwaParam.gif_name = 'q3较近距离动图.gif';
% q3_q5DWAsolve(droneA, droneB, dwaParam, map);
% text(xb - 170, 100, ['(' num2str(xb) ', ' num2str(0) ')'], 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
% img_name = "q3较近距离-" + "无人机B坐标：(" + num2str(xb) + ", 0)" + ".png";
% print(img_name, '-dpng','-r600');

vb = 10;
xb = 1973.49;
map = getQ3map(xb);
[droneA, droneB, dwaParam] = get_param(vb, map, true);
dwaParam.drawTangent = false;
dwaParam.gif = false;
dwaParam.gif_dt = 0.07;
dwaParam.gif_name = 'q3刚好不妨碍动图.gif';
q3_q5DWAsolve(droneA, droneB, dwaParam, map);
text(xb - 400, 100, ['(' num2str(xb) ', ' num2str(0) ')'], 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
img_name = "q3刚好不妨碍-" + "无人机B坐标：(" + num2str(xb) + ", 0)" + ".png";
print(img_name, '-dpng','-r600');

% vb = 10;
% xb = 2080;
% map = getQ3map(xb);
% [droneA, droneB, dwaParam] = get_param(vb, map, true);
% dwaParam.drawTangent = true;
% dwaParam.gif = true;
% dwaParam.gif_dt = 0.07;
% dwaParam.gif_name = 'q3略有妨碍动图.gif';
% q3_q5DWAsolve(droneA, droneB, dwaParam, map);
% text(xb - 350, 100, ['(' num2str(xb) ', ' num2str(0) ')'], 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
% img_name = "q3略有妨碍-" + "无人机B坐标：(" + num2str(xb) + ", 0)" + ".png";
% print(img_name, '-dpng','-r600');

% vb = 10;
% xb = 5000;
% map = getQ3map(xb);
% [droneA, droneB, dwaParam] = get_param(vb, map, true);
% dwaParam.drawTangent = true;
% dwaParam.gif = true;
% dwaParam.gif_dt = 0.06;
% dwaParam.gif_name = 'q3超远距离动图.gif';
% q3_q5DWAsolve(droneA, droneB, dwaParam, map);
% text(xb - 600, 100, ['(' num2str(xb) ', ' num2str(0) ')'], 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
% img_name = "q3超远距离-" + "无人机B坐标：(" + num2str(xb) + ", 0)" + ".png";
% print(img_name, '-dpng','-r600');


%% 问题三地图信息函数
function map = getQ3map(xb)
    % 第二问求切点即可
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
    droneA.headingWeight = 0.1;             % 终点朝向权重
    droneA.dis2goalWeight = 2;            % 终点距离权重
    droneA.StaticDisWeight = 4;           % A逃离静态障碍的意愿
    droneA.DynamicDisWeight = 2;          % A逃离动态障碍的意愿
    droneA.minDis2StaticObstacle = 5 * droneA.v;     % A侦测静态障碍的距离
    droneA.minDis2DynamicObstacle = 5 * droneA.v;    % A侦测动态障碍的距离
    droneA.sampleDuration = t_s(droneA.v);            % 采样时长
    
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
    droneB.headingWeight = 0.1; 
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
    dwaParam.gif = false;
    if Display == true
        dwaParam.displayIter = 10;
    else
        dwaParam.displayIter = inf;
    end
    dwaParam.drawTangent = false;
end





