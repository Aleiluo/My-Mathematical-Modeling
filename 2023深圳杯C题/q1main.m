warning('off')
clc,clear;
close all;
map = getQ1map();

% 定义无人机A参数
droneA.curpos = [-1000, 0, 0, 10, 0];   % [x, y, theta, v, w]
droneA.goal = [3500, 0];                % 终止点坐标
droneA.v = 10;                          % 无人机速度
droneA.a_max = 0;                       % 无人机加速度
droneA.w_max = droneA.v / 30;           % 无人机最大角速度
droneA.alpha_max = 100 * pi /180;         % 无人机最大角加速度
droneA.upper = 1500;
droneA.lower = -1500;
droneA.left = -1100;
droneA.right = 3600;

% 定义无人机B参数
droneB.curpos = [3500, 0, pi, 10, 0];
droneB.goal = [-1000, 0, pi];
droneB.v = 10;
droneB.a_max = 0;  
droneB.w_max = droneB.v / 30;        
droneB.alpha_max = 80 * pi /180; 
droneB.upper = 0;
droneB.lower = -1500;
droneB.left = -1500;
droneB.right = 4000;

% 标准化参数
dwaParam.dt = 0.3;                      % 时间分割间隔
dwaParam.dw = 1.0 * pi /180;            % 角度分辨率
dwaParam.maxIter = 10000;               % 最大迭代次数
dwaParam.headingWeight = 0.1;      % 终点朝向权重
dwaParam.distanceWeight = 3;           % 距离权重
dwaParam.dis2goalWeight = 0.1;
dwaParam.sampleDuration = 11;           % 采样时长
dwaParam.minDis2Obstacle = 50;          % 无人机识别障碍物的最大距离(靠近障碍物了才会识别)
dwaParam.displayIter = 10;              % 若干次迭代更新一次图
dwaParam.gif = false;
dwaParam.gif_name = 'q1动图.gif';
dwaParam.git_dt = 0.07;
%% 运行DWA程序
close all;
res = q1DWAsolve(droneA, droneB, dwaParam, map, true);
print('q1结果.png', '-dpng','-r600');

%% 输出结果
close all;
% fprintf('A耗时%f\nB耗时%f\n', Acost, Bcost);
% 
% 第一张图 - 每一步的角速度柱形图
% figure;
% bar(res.A_w, 1, 'FaceColor', '#26C6DA');
% xlabel('步数');
% ylabel('角速度');
% hold on;  % 保持图形以添加额外的元素
% line([0, length(res.A_w)+1], [0.333, 0.333], 'Color', '#E53935', 'LineStyle', '--', 'LineWidth', 1.3);  % 添加红色虚线
% line([0, length(res.A_w)+1], [-0.333, -0.333], 'Color', '#E53935', 'LineStyle', '--', 'LineWidth', 1.3);  % 添加红色虚线
% legend('角速度', '最大角速度');
% grid on;
% 
% print('q1角速度.png', '-dpng','-r600');

figure;
plot(res.A_dis2sta,'LineWidth',1.7,'Color','#FF7043');
line([0, length(res.A_dis2sta)], [0, 0], 'Color', '#E53935', 'LineStyle', '--', 'LineWidth', 1.3);
ylim([-20, 400]);
xlabel('步数', 'FontSize', 13);
ylabel('到静态障碍物距离', 'FontSize', 13);
hold on;  % 保持图形以添加额外的元素
grid on;
print('q1-A到静态障碍物距离.png', '-dpng','-r600');

figure;
plot(res.AB_dis2o,'LineWidth',1.7,'Color','#66BB6A');
line([0, length(res.AB_dis2o)], [map.R, map.R], 'Color', '#E53935', 'LineStyle', '--', 'LineWidth', 1.3);
ylim([0, 520]);
xlabel('步数', 'FontSize', 13);
ylabel('到原点距离', 'FontSize', 13);
hold on;  % 保持图形以添加额外的元素
grid on;
print('q1-AB连线到静态障碍物距离.png', '-dpng','-r600');


% figure;
% 
% % 绘制第一列数据
% subplot(3, 1, 1); % 创建一个3x1的子图，选择第一个子图
% plot(res.A_score(:, 1),'LineWidth',1.5,'Color','#EF5350');
% xlabel('步数');
% ylabel('朝向得分');
% grid on;
% 
% % 绘制第二列数据
% subplot(3, 1, 2); % 选择第二个子图
% plot(res.A_score(:, 2),'LineWidth',1.5,'Color','#29B6F6');
% xlabel('步数');
% ylabel('避障得分');
% grid on;
% 
% % 绘制第三列数据
% subplot(3, 1, 3); % 选择第三个子图
% plot(res.A_score(:, 3),'LineWidth',1.5,'Color','#FFCA28');
% xlabel('步数');
% ylabel('终点距离得分');
% grid on;
% 
% % 导出图像
% print('q1各项得分(标准化).png', '-dpng', '-r600');

%% 问题一地图信息函数
function map = getQ1map()
    % 第一问地图信息以及无人机B的运动轨迹信息
    map.R = 500;
    map.A = [-1000, 0];
    map.B = [3500, 0];
    C2_B = sqrt(map.B(1)^2 - map.R^2);
    A_C1 = sqrt(map.A(1)^2 - map.R^2);
    kB = sqrt(map.R^2 / (map.B(1)^2 - map.R^2));
    kA = sqrt(map.R^2 / (map.A(1)^2 - map.R^2));
    map.thetaB = abs(atan(kB));
    map.thetaA = abs(atan(kA));
    map.cutB = [map.R * sin(map.thetaB), map.R * cos(map.thetaB)];
    map.cutA = [-map.R * sin(map.thetaA), map.R * cos(map.thetaA)];
    % 圆弧的角度
    delta_theta = abs(acos(dot(map.cutA, map.cutB) / (norm(map.cutA) * norm(map.cutB))));
    Arclen = delta_theta * map.R;
    % 确认分段函数的时间点
    map.t1 = C2_B / 10;
    map.t2 = map.t1 + Arclen / 10;
    map.t3 = map.t2 + A_C1 / 10;
end




