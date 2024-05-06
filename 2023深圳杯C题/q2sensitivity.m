%% 本程序使用DWA算法实现了双无人机带约束的避障
warning('off')
clc,clear;
close all;

map = getQ2map();

% 定义无人机A参数
droneA.curpos = [-1000, 0, 0, 10, 0];   % [x, y, theta, v, w]
droneA.goal = [map.cutC3(1), map.cutC3(2) - 50; ...
               map.cutC4(1), map.cutC4(2) - 100; ...
               3500, 0];                % 对于无人机A使用双目标DWA
% droneA.goal = [3500, 0];                % 对于无人机A使用双目标DWA
droneA.curgoal = 1;                     % 无人机目标(行)
droneA.v = 10;                          % 无人机速度
droneA.a_max = 0;                       % 无人机加速度
droneA.w_max = droneA.v / 30;           % 无人机最大角速度
droneA.alpha_max = 100 * pi /180;       % 无人机最大角加速度
droneA.headingWeight = 0.0000001;        % 终点朝向权重
droneA.dis2goalWeight = 2;              % 终点距离权重
droneA.minDis2StaticObstacle = 50;      % A侦测静态障碍的距离
droneA.StaticDisWeight = 1;             % A逃离静态障碍的意愿
droneA.minDis2DynamicObstacle = 50;     % A侦测动态障碍的距离
droneA.DynamicDisWeight = 1;        % A逃离动态障碍的意愿

% 定义无人机B参数
droneB.curpos = [3500, 0, pi, 10, 0];
droneB.goal = [map.cutC2(1), map.cutC2(2); ...
               -1000, 0];
droneB.curgoal = 1;
droneB.v = 10;
droneB.a_max = 0;
droneB.w_max = droneB.v / 30;        
droneB.alpha_max = 100 * pi /180; 
droneB.headingWeight = 0.00001; 
droneB.dis2goalWeight = 1; 
droneB.minDis2StaticObstacle = 10;
droneB.StaticDisWeight = 2;  
droneB.minDis2DynamicObstacle = 100; 
droneB.DynamicDisWeight = 2;

% 求解器参数
dwaParam.dt = 0.3;                      % 时间分割间隔
dwaParam.dw = 1 * pi /180;            % 角度分辨率
dwaParam.maxIter = 10000;               % 最大迭代次数
dwaParam.sampleDuration = 10.3;           % 采样时长
dwaParam.disJudge = 20;                 % 在终点半径20米内就视为到达了终点
dwaParam.displayIter = 150;              % 若干次迭代更新一次图



%% 采样时长分析

Aarrive_time = [];
Barrive_time = [];
for i = 9.6:0.2:12
    dwaParam.sampleDuration = i;
    [curAarrive_time, curBarrive_time] = q2DWAsolve(droneA, droneB, dwaParam, map);
    curAarrive_time
    curBarrive_time
    Aarrive_time = [Aarrive_time; curAarrive_time];
    Barrive_time = [Barrive_time; curBarrive_time];
end

figure;
plot(9.6:0.2:12, Aarrive_time, 'Color', '#DF1CA7','Marker','x','LineWidth',1.5);
xlabel('采样时间');
ylabel('总耗时');
grid on;

%% B躲避动态障碍的灵敏度分析

Aarrive_time = [];
Barrive_time = [];
for i = 150:50:500
    droneB.minDis2DynamicObstacle = i;
    [curAarrive_time, curBarrive_time] = q2DWAsolve(droneA, droneB, dwaParam, map);
    curAarrive_time
    curBarrive_time
    Aarrive_time = [Aarrive_time; curAarrive_time];
    Barrive_time = [Barrive_time; curBarrive_time];
end

%%
figure;
plot(150:50:400, Aarrive_time(1:end-1), 'Color', '#FF883F','Marker','+','LineWidth',1.5);
xlabel('侦测距离');
ylabel('总耗时');
grid on;

%% 问题二地图信息函数
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




