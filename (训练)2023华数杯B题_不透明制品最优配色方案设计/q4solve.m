clc,clear;
close all;

warning('off');
load KSbase.mat;
KScoef = table2array(readtable('q1coef_poly1.xlsx'));
stimulusData = readtable('./第11题附件/光谱三刺激值加权表.xlsx');
q2targetData = readtable('./第11题附件/样品R值.xlsx');
unitPrice = [60, 65, 63];
baseWeight = 2;
lamb = 400:20:700;

% 定义变量上下界
lb = 0 * ones(3, 1);
ub = 0.05 * ones(3, 1);

% 求解器选项
options = optimoptions(@gamultiobj);
options.PopulationSize = 500;
options.ParetoFraction = 0.6;
%options.PlotFcn = "gaplotpareto";

result = {};
for i = 1:5
    % 计算目标样品的XYZ
    target_XYZ = [0.1 * trapz(lamb, stimulusData.sx .* q2targetData{:, i}), ...
                  0.1 * trapz(lamb, stimulusData.sy .* q2targetData{:, i}), ...
                  0.1 * trapz(lamb, stimulusData.sz .* q2targetData{:, i})];
    % 定义目标函数
    objective = @(vars) [obj(vars, lamb, KScoef, KSbase, stimulusData, target_XYZ); ...
                         sum(baseWeight .* vars .* unitPrice); ...
                         sum(baseWeight .* vars)];
    % 定义色差约束
    nonlcon = @(vars) cons(vars, lamb, KScoef, KSbase, stimulusData, target_XYZ);
    
    curResult = [];
    [curResult(:,1:3), curResult(:,4:6)] = gamultiobj(objective, 3, [], [], [], [], lb, ub, nonlcon, options);
    result{i} = sortrows(curResult, 4);
end

%% 出图出结果

load q4result.mat
sample5result = {};
for i = 1:5
    ind = floor(linspace(1,size(result{i}, 1) - 1, 5));
    sample5result{i} = result{i}(ind, :);
end

% 绘制三维帕累托前沿
close all;
figure;

color = ["#EF5350","#42A5F5","#8BC34A","#FFEE58","#78909C"];

legend_name = [];

for i = 1:5
    deltaE = result{i}(:, 4);
    price = result{i}(:, 5);
    weight = result{i}(:, 6);

    plot3(deltaE, price, weight, 'Color',color(i),'LineWidth',2);
    hold on;
    scatter3(sample5result{i}(:,4), sample5result{i}(:,5), sample5result{i}(:,6), ...
            'Marker','pentagram','MarkerEdgeColor',color(i),'MarkerFaceColor','none', ...
            'LineWidth',0.7,'SizeData',70);
    hold on;
    legend_name = [legend_name; "样品" + num2str(i);""];
end
legend(legend_name);

xlabel('色差');
ylabel('价格');
zlabel('重量');
grid on;

%% 绘制三维柱状图
close all;

color = ["#E0E0E0"; "#EF5350"; "#FFEE58"; "#42A5F5"];


figure;
set(gcf, 'Renderer', 'painters');
for i = 1:5
% 绘制三维柱状图
b = bar3([0.06 * (i - 1)*ones(5,1),sample5result{i}(:, 1:3)], 'stacked');
hold on;

b(1).FaceColor = color(1);
b(1).EdgeAlpha = 0.3;
b(2).FaceColor = color(2);
b(3).FaceColor = color(3);
b(4).FaceColor = color(4);
% 调整位置
for j = 1:length(b)
    for k = 1:length(b(j).XData)
        b(j).XData(k, :) = b(j).XData(k, :) + 1 * (i - 1);
    end
end

end

pbaspect([1 1 0.7]);
xlim([0,6]);
ylim([0,6]);

categories = {};
for i = 1:5
    categories{i} = "样品" + num2str(i);
end
formula = {};
for i = 1:5
    formula{i} = "配方" + num2str(i);
end
ax = gca;
ax.XTick = 1:5;
ax.XTickLabel = categories;
ax.YTickLabel = formula;

%% 绘制结果折线图
close all;

% 色差图
color = ["#EC407A","#673AB7","#00BCD4","#8BC34A","#FF5722"];
marker = ['+',"square",'^',"pentagram",'*'];
linestyle = ['-' ,"--","-.",':','-'];

figure;
for i=1:5
    plot(sample5result{i}(:,4),'LineStyle',linestyle(i),'Marker',marker(i),'Color',color(i),'LineWidth',1.5);
    hold on;
end
xticks(1:5);
xticklabels(formula);
ylabel('\Delta E^*');
legend(categories,'location','northwest');

% 价格图
figure;
for i=1:5
    plot(sample5result{i}(:,5),'LineStyle',linestyle(i),'Marker',marker(i),'Color',color(i),'LineWidth',1.5);
    hold on;
end
xticks(1:5);
xticklabels(formula);
ylabel('总价格');
legend(categories,'location','northwest');

% 重量图
figure;
for i=1:5
    plot(sample5result{i}(:,6),'LineStyle',linestyle(i),'Marker',marker(i),'Color',color(i),'LineWidth',1.5);
    hold on;
end
xticks(1:5);
xticklabels(formula);
ylabel('总重量');
legend(categories,'location','northwest');

%% 绘制雷达图

% Initialize data points
D1 = [5 3 9 1 2];
D2 = [5 8 7 2 9];
D3 = [8 2 1 4 6];
P = [D1; D2; D3];

% Spider plot
spider_plot_R2019b(P,...
    'AxesLabels', {'S1', 'S2', 'S3', 'S4', 'S5'},...
    'AxesInterval', 2,...
    'FillOption', {'on', 'on', 'off'},...
    'FillTransparency', [0.2, 0.1, 0.1]);

%% 函数
function deltaE = obj(vars, lamb, KScoef, KSbase, stimulusData, target_XYZ)
    % 计算K/S
    % 线性拟合公式
    mixKS = KSbase + ...
            vars(1) .* (KScoef(:,1) .* vars(1) + KScoef(:,2)) + ...
            vars(2) .* (KScoef(:,3) .* vars(2) + KScoef(:,4)) + ...
            vars(3) .* (KScoef(:,5) .* vars(3) + KScoef(:,6));

    % 计算R
    R = 1 + mixKS - sqrt(mixKS.^2 + 2 .* mixKS);
    % 计算XYZ
    opt_XYZ = [0.1 * trapz(lamb, stimulusData.sx .* R), ...
               0.1 * trapz(lamb, stimulusData.sy .* R), ...
               0.1 * trapz(lamb, stimulusData.sz .* R)];

    % 计算色差
    % 标准XYZ
    X0 = 94.83;
    Y0 = 100;
    Z0 = 107.38;

    % 计算目标样品 L* a* b*       记为样本1：sp1
    targetRatio = target_XYZ ./ [X0, Y0, Z0];
    if all(targetRatio > 0.008856)
        [L1, a1, b1] = ParamFcn1(targetRatio);
    else
        [L1, a1, b1] = ParamFcn2(targetRatio);
    end

    % 计算优化样品 L* a* b*       记为样本2：sp2
    optRatio = opt_XYZ ./ [X0, Y0, Z0];
    if all(optRatio > 0.008856)
        [L2, a2, b2] = ParamFcn1(optRatio);
    else
        [L2, a2, b2] = ParamFcn2(optRatio);
    end

    deltaE = sqrt((L1 - L2)^2 + (a1 - a2)^2 + (b1 - b2)^2);
end

function [c, ceq] = cons(vars, lamb, KScoef, KSbase, stimulusData, target_XYZ)
    % 计算K/S
    % 线性拟合公式
    mixKS = KSbase + ...
            vars(1) .* (KScoef(:,1) .* vars(1) + KScoef(:,2)) + ...
            vars(2) .* (KScoef(:,3) .* vars(2) + KScoef(:,4)) + ...
            vars(3) .* (KScoef(:,5) .* vars(3) + KScoef(:,6));

    % 计算R
    R = 1 + mixKS - sqrt(mixKS.^2 + 2 .* mixKS);
    % 计算XYZ
    opt_XYZ = [0.1 * trapz(lamb, stimulusData.sx .* R), ...
               0.1 * trapz(lamb, stimulusData.sy .* R), ...
               0.1 * trapz(lamb, stimulusData.sz .* R)];

    % 计算色差
    % 标准XYZ
    X0 = 94.83;
    Y0 = 100;
    Z0 = 107.38;

    % 计算目标样品 L* a* b*       记为样本1：sp1
    targetRatio = target_XYZ ./ [X0, Y0, Z0];
    if all(targetRatio > 0.008856)
        [L1, a1, b1] = ParamFcn1(targetRatio);
    else
        [L1, a1, b1] = ParamFcn2(targetRatio);
    end

    % 计算优化样品 L* a* b*       记为样本2：sp2
    optRatio = opt_XYZ ./ [X0, Y0, Z0];
    if all(optRatio > 0.008856)
        [L2, a2, b2] = ParamFcn1(optRatio);
    else
        [L2, a2, b2] = ParamFcn2(optRatio);
    end

    deltaE = sqrt((L1 - L2)^2 + (a1 - a2)^2 + (b1 - b2)^2);
    c = deltaE - 1;
    ceq = [];
end

function [L, a, b] = ParamFcn1(ratio)
    % ratio(0) = X / X0
    % ratio(1) = Y / Y0
    % ratio(2) = Z / Z0
    L = 116 * ratio(2)^(1/3) - 16;
    a = 500 * (ratio(1)^(1/3) - ratio(2)^(1/3));
    b = 200 * (ratio(2)^(1/3) - ratio(3)^(1/3));
end

function [L, a, b] = ParamFcn2(ratio)
    L = 903.3 * ratio(2);
    a = 3893.5 * (ratio(1) - ratio(2));
    b = 1557.4 * (ratio(2) - ratio(3));
end
