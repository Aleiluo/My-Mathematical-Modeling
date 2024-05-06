
clear;

%%
close all;
popSize = 70;
maxIter = 150;
dim = 9;
% Arlim: 镜子分区极值
% (镜子长宽高(3)，两个抛物线的信息(4[a,b,c,d]), A_sf, Arlim)

% 第一环最小镜子数
N0_min = ceil(2 * 102.5 / (8 * sqrt(2) + 5));
lb = [2, 2, 2, 0.0001, -350, -0.01,  200, 1, 0.5];
ub = [8, 8, 6, 0.01,   0   , -0.001, 350, 1.9, 1.7];
fobj = @(vars) q2solve(vars);

[Xbest, Fbest, gen_fitness] = q2ISSA(fobj, dim, lb, ub, popSize, maxIter);

% 模拟运行
%fobj([5, 5, 5, 0.05, 0, -0.01, 350, 3, 1.1]);

%fobj([6, 6, 4, 0.0001, -350, -0.001, 350, 1.7, 1]);

%% 用于坐标保存

fobj(Xbest);

%% 第二问计算实际真值
load 问题二坐标.mat

% 参数设置

% --- 时间参数 ---
q2_day_number = 12;
q2_times_number = 5;
% 12个月，5个时间点
q2_days = datetime(2023, 1:q2_day_number, 21);
% 一天5个时间点
q2_daytimes = [9, 10.5, 12, 13.5, 15];
% 春分日期
SpringEquinox = datetime(2023, 3, 21);

% --- 反射镜与接收塔参数 ---
param.len = Xbest(1);
param.width = Xbest(2);
param.height = Xbest(3);
param.Hz = 80;
% 集热器高度
param.col_h = 8;
param.col_d = 7;
% 假设定日镜距离大于一定值，两者必不可能发生遮蔽现象
param.max_dis = 30;
param.eta_ref = 0.92;

% --- 遮蔽损失参数 ---
subs = 12;
sub_rad = 2 * pi / subs;
full_info_cnt = 0;
full_info = {};
DNI = {};
sub_mirrors = {};


%% 问题求解
mirror_number = size(mirrors, 1);
% 初始化镜面信息
for i = 1:subs
    sub_mirrors{i} = [];
end
% 坐标划分
for i = 1:mirror_number
    % 使用四象限反正切计算坐标夹角
    pos = ceil((atan2(mirrors(i, 2), mirrors(i, 1)) + pi) / sub_rad);
    sub_mirrors{pos} = [sub_mirrors{pos}; ...
        [mirrors(i, 1), mirrors(i, 2), param.height, param.len, param.width, sqrt(mirrors(i, 1)^2 + mirrors(i, 2)^2)]];
end
% 对立面点进行距离排序
for i = 1:subs
    if isempty(sub_mirrors{i})
        continue
    end
    sub_mirrors{i} = sortrows(sub_mirrors{i}, 6, "descend");
end

k = 0;
q2_res = zeros(q2_day_number, 7);
for i = 1:q2_day_number
    dayRes_mean = zeros(5, 7);
    D = days(q2_days(i) - SpringEquinox);
    for j = 1:q2_times_number
        ST = q2_daytimes(j);
        disp(q2_days(i));
        disp(ST);
        % 求此刻的DNI、太阳的高度角、方位角
        param = calDNI(D, ST , param);

        % 求此刻所有镜子的方位角和俯仰角
        sub_mirrors = cal_mirror_info(sub_mirrors, param);

        % 计算各个指标
        sub_mirrors = cal_target(sub_mirrors, param);
        
        % 记录信息
        full_info_cnt = full_info_cnt + 1;
        full_info{full_info_cnt} = sub_mirrors;
        DNI{full_info_cnt} = param.DNI;
        % 求均值
        timeRes_mean = zeros(1, 7);
        tot_area = 0;
        for k = 1:subs
            if isempty(sub_mirrors{k})
                continue
            end
            % 第一列：光学效率
            timeRes_mean(1) = timeRes_mean(1) + sum(sub_mirrors{k}(:, 32));
            % 第二列：余弦效率
            timeRes_mean(2) = timeRes_mean(2) + sum(sub_mirrors{k}(:, 29));
            % 第三列：遮挡效率
            timeRes_mean(3) = timeRes_mean(3) + sum(sub_mirrors{k}(:, 27));
            % 第四列：截断效率
            timeRes_mean(4) = timeRes_mean(4) + sum(sub_mirrors{k}(:, 28));
            % 第五列：大气效率
            timeRes_mean(5) = timeRes_mean(5) + sum(sub_mirrors{k}(:, 30));
            % 第六列：总功率
            timeRes_mean(6) = timeRes_mean(6) + param.DNI * sum(sub_mirrors{k}(:, 33));
            % 总面积
            tot_area = tot_area + sum(sub_mirrors{k}(:, 4) .* sub_mirrors{k}(:, 5));
        end
        % 计算一时刻的平均效率
        timeRes_mean(1:5) = timeRes_mean(1:5) / mirror_number;
        % 第七列：单位面积功率
        timeRes_mean(7) = timeRes_mean(6) / tot_area;
        
        dayRes_mean(j, :) = timeRes_mean;
    end
    q2_res(i, :) = mean(dayRes_mean, 1);
end

full_info = reshape(full_info, 12, 5);
DNI = reshape(DNI, 12, 5);
%% 出图出结果
load q2结果.mat
close all;
% 绘制迭代收敛图
plot(gen_fitness(1:150),Color='#AB47BC',LineWidth=1.8);
xlabel('迭代次数');
ylabel('$Opt$','Interpreter','latex',Rotation=0,FontSize=12);
legend('单位面积热功率倒数');
saveas(gcf, 'q2收敛曲线.svg');

%% 每一块镜子的年平均，并绘制热力图
close all;
load q2结果.mat

A = cat2one(full_info{1, 1});
tot_area = sum(A(:, 4) .* A(:, 5));
A(:, 6:end) = 0;
for i = 1:size(full_info,1)
    for j = 1:size(full_info, 2)
        B = cat2one(full_info{i, j});
        A(:, 6) = A(:, 6) + DNI{i, j} * B(:, 33);
    end
end
A(:, 6) = A(:, 6) ./ (size(full_info,1) .* size(full_info,2) .* ...
    A(:, 4) .* A(:, 5));

% 绘制热力图
% 绘制每一片镜子的总效率
figure;
scatter(A(:,1), A(:,2), 10, A(:,6), 'filled');
colorbar;
hold on;
x = linspace(-400, 400, 100);
plot(x, Xbest(4) .* x.^2 + Xbest(5), LineWidth=2,LineStyle="--",Color='#E91E63');
plot(x, Xbest(6) .* x.^2 + Xbest(7), LineWidth=2,LineStyle="--",Color='#D4E157');
axis equal;
xlabel('$x$','Interpreter','latex',FontSize=13);
ylabel('$y$','Interpreter','latex','Rotation',0,FontSize=13);
xlim([-400, 400]);
ylim([-70, 400]);
saveas(gcf, "q2年均单位面积热功率.svg");

%% 第二问三个表

load q2结果.mat
q2_res(:, 6) = q2_res(:, 6) / 1e3;
q2_sheet1 = q2_res(:, [1:4, 7]);
q2_sheet2 = mean(q2_res(:, [1:4, 6, 7]));
writematrix(q2_sheet1, 'q2表一.xlsx');
writematrix(q2_sheet2, 'q2表二.xlsx');

%% 函数
function A = cat2one(sub_mirrors)
    A = [];
    subs = length(sub_mirrors);
    for i = 1:subs
        A = [A; sub_mirrors{i}];
    end
end