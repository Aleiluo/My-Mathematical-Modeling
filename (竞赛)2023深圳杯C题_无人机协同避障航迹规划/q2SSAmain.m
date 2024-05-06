clc,clear;

%% 麻雀优化算法

close all;
map = getQ2map();

% 获取默认参数
[droneA, droneB, dwaParam] = get_param();

popSize = 60;
maxIter = 300;
dim = 15;
lb = [0,  0,  0,  0,  10, 10, -pi, ...
      0,  0,  0,  0,  10, 10, 0, ...
      8];
ub = [30, 30, 30, 30, 60, 60, -pi/2, ...
      30, 30, 30, 30, 60, 60, pi/2, ...
      15];
fobj = @(x) q2DWAforSSA(x, droneA, droneB, dwaParam, map);
[Xbest, Fbest, gen_x, gen_fitness] = par_ISSA(fobj, [], dim, lb, ub, popSize, maxIter, true);

%% 使用求解的参数
load q2res_603.215.mat
map = getQ2map();
% 获取默认参数
[droneA, droneB, dwaParam] = get_param(Xbest, map);
% gif参数设置
dwaParam.gif = false;
dwaParam.gif_name = 'q2_动图.gif';
dwaParam.gif_dt = 0.08;
dwaParam.maxIter = 2000;
dwaParam.displayIter = 5;
dwaParam.drawTangent = false;
res = q2DWAsolve(droneA, droneB, dwaParam, map);
print('q2结果.png', '-dpng','-r600');

%% 
map = getQ2map();
% 获取默认参数
[droneA, droneB, dwaParam] = test_param(map);
% gif参数设置
dwaParam.gif = false;
dwaParam.gif_name = 'q2_动图.gif';
dwaParam.gif_dt = 0.08;
dwaParam.maxIter = 2000;
dwaParam.displayIter = 5;
dwaParam.drawTangent = true;
res = q2DWAsolve(droneA, droneB, dwaParam, map);
res.A_time
res.B_time

%% 正确性检验
close all;

figure;
bar(res.A_w, 1, 'FaceColor', '#EC407A');
xlabel('步数', 'FontSize', 13);
ylabel('角速度', 'FontSize', 13);
hold on;  % 保持图形以添加额外的元素
line([0, length(res.A_w)+1], [0.333, 0.333], 'Color', '#E53935', 'LineStyle', '--', 'LineWidth', 1.3);  % 添加红色虚线
line([0, length(res.A_w)+1], [-0.333, -0.333], 'Color', '#E53935', 'LineStyle', '--', 'LineWidth', 1.3);  % 添加红色虚线
legend('角速度', '最大角速度');
grid on;

print('q2无人机A角速度.png', '-dpng','-r600');

figure;
bar(res.B_w, 1, 'FaceColor', '#29B6F6');
xlabel('步数', 'FontSize', 13);
ylabel('角速度', 'FontSize', 13);
hold on;  % 保持图形以添加额外的元素
line([0, length(res.B_w)+1], [0.333, 0.333], 'Color', '#E53935', 'LineStyle', '--', 'LineWidth', 1.3);  % 添加红色虚线
line([0, length(res.B_w)+1], [-0.333, -0.333], 'Color', '#E53935', 'LineStyle', '--', 'LineWidth', 1.3);  % 添加红色虚线
legend('角速度', '最大角速度');
grid on;

print('q2无人机B角速度.png', '-dpng','-r600');

% 到静态、动态障碍物距离
figure;
plot(res.A_dis2sta,'LineWidth',1.7,'Color','#FF7043');
line([0, length(res.A_dis2sta)], [0, 0], 'Color', '#E53935', 'LineStyle', '--', 'LineWidth', 1.3);
xlim([0, 800]);
ylim([-20, 300]);
xlabel('步数', 'FontSize', 13);
ylabel('到静态障碍物距离', 'FontSize', 13);
hold on;  % 保持图形以添加额外的元素
grid on;
print('q2-A到静态障碍物距离.png', '-dpng','-r600');

figure;
plot(res.A_dis2sta,'LineWidth',1.7,'Color','#66BB6A');
line([0, length(res.A_dis2sta)], [0, 0], 'Color', '#E53935', 'LineStyle', '--', 'LineWidth', 1.3);
xlim([210, 270]);
ylim([-20,20]);
xlabel('步数', 'FontSize', 13);
ylabel('到原点距离', 'FontSize', 13);
hold on;  % 保持图形以添加额外的元素
grid on;
print('q2-A到静态障碍物距离(缩放图).png', '-dpng','-r600');

figure;
plot(res.AB_dis2o,'LineWidth',1.7,'Color','#66BB6A');
line([0, length(res.AB_dis2o)], [map.R, map.R], 'Color', '#E53935', 'LineStyle', '--', 'LineWidth', 1.3);
xlim([100, 800]);
ylim([390, 510]);
xlabel('步数', 'FontSize', 13);
ylabel('到原点距离', 'FontSize', 13);
hold on;  % 保持图形以添加额外的元素
grid on;
print('q2-AB连线到静态障碍物距离.png', '-dpng','-r600');

figure;
plot(res.AB_dis2o,'LineWidth',1.7,'Color','#66BB6A');
line([0, length(res.AB_dis2o)], [map.R, map.R], 'Color', '#E53935', 'LineStyle', '--', 'LineWidth', 1.3);
xlim([585, 625]);
ylim([492,502]);
xlabel('步数', 'FontSize', 13);
ylabel('到原点距离', 'FontSize', 13);
hold on;  % 保持图形以添加额外的元素
grid on;
print('q2-AB连线到静态障碍物距离(缩放图).png', '-dpng','-r600');



%% 麻雀算法1到30代最优解图
load q2res_603.215.mat
map = getQ2map();

image_counts = 0;
im = {};
i_range = [1, 5:5:30, 40:20:300];
for i = i_range
    [droneA, droneB, dwaParam] = get_param(gen_x(i,:), map);
    dwaParam.maxIter = 2000;
    dwaParam.displayIter = 10;
    dwaParam.drawTangent = false;
    res = q2DWAsolve(droneA, droneB, dwaParam, map);
    title("第" + num2str(i) + "代, 无人机A总耗时：" + num2str(res.A_time) + "秒", 'FontSize', 16);
    image_counts = image_counts + 1;
    im{image_counts} = frame2im(getframe(res.fig));
end
%%
gif_dt = [repmat(2, 1, 8), repmat(1, 1, 13)];

for i = 1:image_counts
    [Image, Image_map] = rgb2ind(im{i},256);
    if i == 1
        imwrite(Image,Image_map,"q2迭代线路图.gif","gif","LoopCount",Inf,"DelayTime",gif_dt(i));
    else
        imwrite(Image,Image_map,"q2迭代线路图.gif","gif","WriteMode","append","DelayTime",gif_dt(i));
    end
end

%% 迭代图
% 迭代收敛图
load q2res_603.215.mat
figure;
plot(gen_fitness,'Color','#9CCC65','LineWidth',2);
xlabel('迭代次数', FontSize=12);
ylabel('无人机A总耗时', FontSize=12);
%print('q2收敛曲线.png', '-dpng','-r600');


%% 三种较劣情况
close all;
map = getQ2map();
% 情况一：折返，原因：到终点距离权重太低，heading太大，导致无人机不肯360度转圈。
% x1 = [1, 0.1, 5, 5, 50, 50, -pi/2, ...
%       0.1, 0.2, 5, 5, 50, 50, pi/2, 11];
% [droneA, droneB, dwaParam] = get_param(x1, map);
% dwaParam.gif = false;
% dwaParam.maxIter = 2000;
% dwaParam.displayIter = 10;
% dwaParam.drawTangent = false;
% res = q2DWAsolve(droneA, droneB, dwaParam, map);
% print("q2折返耗时" + num2str(res.A_time) + ".png", '-dpng','-r600');

% 情况二：B躲避无人机A，原因：无人机B躲避动态障碍物权重过大
% x2 = [0.1, 0.1, 5, 5, 50, 50, -pi/2, ...
%       0.1, 0.1, 5, 5, 50, 400, pi/2, 11];
% [droneA, droneB, dwaParam] = get_param(x2, map);
% dwaParam.gif = false;
% dwaParam.maxIter = 2000;
% dwaParam.displayIter = 10;
% dwaParam.drawTangent = false;
% res = q2DWAsolve(droneA, droneB, dwaParam, map);
% print("q2-B躲避无人机A耗时" + num2str(res.A_time) + ".png", '-dpng','-r600');
% 
% % 情况三：无人机A走入死角，原因：t_s过小，无人机无法探测到风险(生成动图)
x3 = [0.1, 0.2, 5, 5, 50, 50, -pi/2, ...
      0.1, 0.2, 5, 5, 50, 50, pi/2, 9];
[droneA, droneB, dwaParam] = get_param(x3, map);
dwaParam.gif = false;
dwaParam.maxIter = 2000;
dwaParam.displayIter = 10;
dwaParam.drawTangent = true;
res = q2DWAsolve(droneA, droneB, dwaParam, map);
print('q2无人机A走入死角.png', '-dpng','-r600');


%% 函数
function map = getQ2map()
    % 第二问求切点即可
    map.R = 500;
    map.A = [-1000, 0];
    map.B = [3500, 0];
    kB = sqrt(map.R^2 / (map.B(1)^2 - map.R^2));
    kA = sqrt(map.R^2 / (map.A(1)^2 - map.R^2));
    map.thetaB = abs(atan(kB));
    map.thetaA = abs(atan(kA));
    map.cutC1 = [-map.R * sin(map.thetaA), map.R * cos(map.thetaA)];
    map.cutC2 = [map.R * sin(map.thetaB), map.R * cos(map.thetaB)];
    map.cutC3 = [-map.R * sin(map.thetaA), -map.R * cos(map.thetaA)];
    map.cutC4 = [map.R * sin(map.thetaB), -map.R * cos(map.thetaB)];
end

function [droneA, droneB, dwaParam] = get_param(x, map)
    % 固定参数设置

    % 定义无人机A参数
    droneA.name = 'A';
    droneA.curpos = [-1000, 0, 0, 10, 0];   % [x, y, theta, v, w]
    % droneA.goal = [map.cutC3(1), map.cutC3(2); ...
    %                map.cutC4(1), map.cutC4(2); ...
    %                3500, 0];                % 对于无人机A使用双目标DWA
    droneA.goal = [0, 0; ...
                   3500, 0];
    droneA.curgoal = 1;                     % 无人机目标(行)
    droneA.v = 10;                          % 无人机速度
    droneA.a_max = 0;                       % 无人机加速度
    droneA.w_max = droneA.v / 30;           % 无人机最大角速度
    droneA.alpha_max = 100 * pi /180;       % 无人机最大角加速度
    
    % 定义无人机B参数
    droneB.name = 'B';
    droneB.curpos = [3500, 0, pi, 10, 0];
    % droneB.goal = [map.cutC2(1), map.cutC2(2); ...
    %                map.cutC1(1), map.cutC1(2); ...
    %                -1000, 0];
    droneB.goal = [0, 0; ...
                   -1000, 0];
    droneB.curgoal = 1;
    droneB.v = 10;
    droneB.a_max = 0;
    droneB.w_max = droneB.v / 30;        
    droneB.alpha_max = 100 * pi /180; 
    
    % 求解器参数
    % 注意：限制最大迭代次数很重要，能大幅提高并行求解效率
    dwaParam.dt = 0.5;                     % 时间分割间隔
    dwaParam.dw = 1 * pi /180;             % 角度分辨率
    dwaParam.maxIter = round(700 / dwaParam.dt);
    dwaParam.displayIter = inf;
    dwaParam.drawTangent = true;
    dwaParam.gif = false;

    % 传入了麻雀算法的参数
    if nargin > 0
        % 无人机A
        droneA.headingWeight = x(1);             % 终点朝向权重
        droneA.dis2goalWeight = x(2);            % 终点距离权重
        droneA.StaticDisWeight = x(3);           % A逃离静态障碍的意愿
        droneA.DynamicDisWeight = x(4);          % A逃离动态障碍的意愿
        droneA.minDis2StaticObstacle = x(5);     % A侦测静态障碍的距离
        droneA.minDis2DynamicObstacle = x(6);    % A侦测动态障碍的距离
        droneA.goal(1, :) = [map.R * cos(x(7)), map.R * sin(x(7))];
        droneA.sampleDuration = x(15);            % 采样时长
        
        % 无人机B
        droneB.headingWeight = x(8); 
        droneB.dis2goalWeight = x(9); 
        droneB.StaticDisWeight = x(10);
        droneB.DynamicDisWeight = x(11);
        droneB.minDis2StaticObstacle = x(12);
        droneB.minDis2DynamicObstacle = x(13); 
        droneB.goal(1, :) = [map.R * cos(x(14)), map.R * sin(x(14))];
        droneB.sampleDuration = x(15);
    end

end

function [droneA, droneB, dwaParam] = test_param(map)
    % 固定参数设置

    % 定义无人机A参数
    droneA.name = 'A';
    droneA.curpos = [-1000, 0, 0, 10, 0];   % [x, y, theta, v, w]
    droneA.goal = [0, 0; ...
                   500 * cos(-3 * pi / 180), 500 * sin(-3 * pi / 180); ...
                   3500, 0];
    droneA.curgoal = 1;                     % 无人机目标(行)
    droneA.v = 10;                          % 无人机速度
    droneA.a_max = 0;                       % 无人机加速度
    droneA.w_max = droneA.v / 30;           % 无人机最大角速度
    droneA.alpha_max = 100 * pi /180;       % 无人机最大角加速度
    
    % 定义无人机B参数
    droneB.name = 'B';
    droneB.curpos = [3500, 0, pi, 10, 0];
    droneB.goal = [0, 0; ...
                   -1000, 0];
    droneB.curgoal = 1;
    droneB.v = 10;
    droneB.a_max = 0;
    droneB.w_max = droneB.v / 30;        
    droneB.alpha_max = 100 * pi /180; 
    
    % 求解器参数
    % 注意：限制最大迭代次数很重要，能大幅提高并行求解效率
    dwaParam.dt = 0.5;                     % 时间分割间隔
    dwaParam.dw = 0.1 * pi /180;             % 角度分辨率
    dwaParam.maxIter = round(700 / dwaParam.dt);
    dwaParam.displayIter = inf;
    dwaParam.drawTangent = false;
    dwaParam.gif = false;

    % 传入了麻雀算法的参数
    theta_A = -pi / 2;
    droneA.headingWeight = 0.2;             % 终点朝向权重
    droneA.dis2goalWeight = 0.2;            % 终点距离权重
    droneA.StaticDisWeight = 3;           % A逃离静态障碍的意愿
    droneA.DynamicDisWeight = 3;          % A逃离动态障碍的意愿
    droneA.minDis2StaticObstacle = 40;     % A侦测静态障碍的距离
    droneA.minDis2DynamicObstacle = 40;    % A侦测动态障碍的距离
    droneA.goal(1, :) = [map.R * cos(theta_A), map.R * sin(theta_A)];
    droneA.sampleDuration = 11;            % 采样时长
    
    % 无人机B
    theta_B = pi / 2;
    droneB.headingWeight = 0.1; 
    droneB.dis2goalWeight = 0.1; 
    droneB.StaticDisWeight = 5;
    droneB.DynamicDisWeight = 5;
    droneB.minDis2StaticObstacle = 30;
    droneB.minDis2DynamicObstacle = 30;
    droneB.goal(1, :) = [map.R * cos(theta_B), map.R * sin(theta_B)];
    droneB.sampleDuration = 11;

end