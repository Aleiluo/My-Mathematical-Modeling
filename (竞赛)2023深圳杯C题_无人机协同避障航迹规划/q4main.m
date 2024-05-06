clc,clear
%% 速度分析

vb_range = 10:0.01:30;
vb_len = length(vb_range);
Aarrive_time = zeros(1, vb_len);
Barrive_time = zeros(1, vb_len);
map = getQ4map(3500);

for i = 1:vb_len
    vb = vb_range(i);
    fprintf("无人机B速度：%f\n", vb);
    [droneA, droneB, dwaParam] = get_param(vb, map, false);
    res = q3_q5DWAsolve(droneA, droneB, dwaParam, map);
    Aarrive_time(i) = res.A_time;
    Barrive_time(i) = res.B_time;
end

%%
load q4res2.mat
close all;
% 拐点坐标
p = [21.12, 471.425];

figure;
plot(vb_range, Aarrive_time, 'Color', '#B0737B','LineWidth',1.5);
hold on;
plot(vb_range, Barrive_time, 'Color', '#004895','LineWidth',1.5);
plot(p(1), p(2), 'kx', 'MarkerSize', 10, 'LineWidth', 1.5);
text(p(1) + 0.2, p(2) - 15, ['拐点坐标: (' num2str(p(1)) ', ' num2str(p(2)) ')'], 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
xlabel('V_B');
ylabel('总耗时');
legend('无人机A','无人机B','耗时拐点');
grid on;

print('q4无人机B速度-耗时曲线.png', '-dpng','-r600');

%% 绘出几种不同情况
% xb = 3500;
% vb = 15;
% map = getQ4map(xb);
% [droneA, droneB, dwaParam] = get_param(vb, map, true);
% dwaParam.drawTangent = false;
% dwaParam.gif = false;
% dwaParam.gif_dt = 0.07;
% dwaParam.gif_name = 'q4较慢速度动图.gif';
% q3_q5DWAsolve(droneA, droneB, dwaParam, map);
% img_name = "q4较慢速度-" + "无人机B速度：" + num2str(vb) + ".png";
% print(img_name, '-dpng','-r600');
% 
% vb = 21.12;
% map = getQ4map(xb);
% [droneA, droneB, dwaParam] = get_param(vb, map, true);
% dwaParam.drawTangent = false;
% dwaParam.gif = false;
% dwaParam.gif_dt = 0.07;
% dwaParam.gif_name = 'q4速度刚好动图.gif';
% q3_q5DWAsolve(droneA, droneB, dwaParam, map);
% img_name = "q4速度刚好-" + "无人机B速度：" + num2str(vb) + ".png";
% print(img_name, '-dpng','-r600');

vb = 40;
map = getQ4map(xb);
[droneA, droneB, dwaParam] = get_param(vb, map, true);
dwaParam.drawTangent = false;
dwaParam.gif = false;
dwaParam.gif_dt = 0.07;
dwaParam.gif_name = 'q4速度较快动图.gif';
q3_q5DWAsolve(droneA, droneB, dwaParam, map);
img_name = "q4速度较快-" + "无人机B速度：" + num2str(vb) + ".png";
print(img_name, '-dpng','-r600');

%% 问题四地图信息函数
function map = getQ4map(xb)
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
    droneA.StaticDisWeight = 3;           % A逃离静态障碍的意愿
    droneA.DynamicDisWeight = 3;          % A逃离动态障碍的意愿
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
    droneB.StaticDisWeight = 3;
    droneB.DynamicDisWeight = 3;
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

