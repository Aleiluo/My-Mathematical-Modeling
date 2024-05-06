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
options.PopulationSize = 600;
options.ParetoFraction = 0.5;
%options.PlotFcn = "gaplotpareto";

result = {};
for i = 1:10
    % 计算目标样品的XYZ
    target_XYZ = [0.1 * trapz(lamb, stimulusData.sx .* q2targetData{:, i}), ...
                  0.1 * trapz(lamb, stimulusData.sy .* q2targetData{:, i}), ...
                  0.1 * trapz(lamb, stimulusData.sz .* q2targetData{:, i})];
    % 定义目标函数
    objective = @(vars) [obj(vars, lamb, KScoef, KSbase, stimulusData, target_XYZ); sum(baseWeight .* vars .* unitPrice)];
    % 定义色差约束
    nonlcon = @(vars) cons(vars, lamb, KScoef, KSbase, stimulusData, target_XYZ);
    
    curResult = [];
    [curResult(:,1:3), curResult(:,4:5)] = gamultiobj(objective, 3, [], [], [], [], lb, ub, nonlcon, options);
    result{i} = sortrows(curResult, 4);
end

%% 绘图出结果

load q3result;

rng(3);
quality = 0.2;

close all;
figure;
hold on;
xlabel('色差');
ylabel('价格');
line_styles = {'-', '--', '-.'};
color = ["#E53935","#8E24AA","#3949AB","#039BE5","#00897B","#7CB342","#FDD835","#FB8C00","#6D4C41","#546E7A"];

legend_name = [];
sample_result = [];

for i = 1:10
    deltaE = result{i}(:,4);
    price = result{i}(:,5);
    [~, index1] = max(deltaE(find(deltaE < quality)));
%     diff_price = abs(diff(diff(smoothdata(price,"gaussian",13))));
%     diff_price(1:10) = 0;
%     diff_price(end) = 0;
%     [~, index2] = max(diff_price);
%     index = min([index1, index2]);

    random_style = line_styles{randi(length(line_styles))};

    plot(deltaE, price,'LineWidth',1.5,'LineStyle',random_style,'Color',color(i));
    scatter(deltaE(index1), price(index1),'Marker','pentagram','MarkerEdgeColor',color(i),'MarkerFaceColor',color(i));
    legend_name = [legend_name; "样品" + num2str(i); ""];
    
    sample_result(i,:) = result{i}(index1,:);
end
legend(legend_name);


% 绘制分段式柱状图
figure;
b1 = bar(sample_result(:,1:3), 'stacked','EdgeColor',"#8D6E63",'LineWidth',1.2);

b1(1).FaceColor = "#F44336";
b1(2).FaceColor = "#FFEB3B";
b1(3).FaceColor = "#2196F3";

categories = {};
for i = 1:10
    categories{i} = "样品" + num2str(i);
end

% 添加标签和标题
set(gca, 'XTickLabel', categories);
ylabel('浓度');
legend('红色浓度', '黄色浓度', '蓝色浓度','Location','northwest');
grid on;

% % 绘制色差柱状图
% figure;
% b2 = bar(sample_result(:,4),'FaceColor',"#CDDC39",'EdgeColor',"#8D6E63",'LineWidth',1.2);
% 
% set(gca, 'XTickLabel', categories);
% ylabel('\Delta E^*');
% grid on;
% 
% % 获取柱状图的X坐标位置
% xPositions = b2.XEndPoints;
% 
% % 获取柱状图的高度（数据大小）
% barHeights = b2.YData;
% 
% % 设置方框参数
% boxWidth = 0.72; % 方框的宽度
% boxHeight = 0.015; % 方框的高度
% boxColor = [0.7 0.7 0.7]; % 方框的颜色
% 
% % 在每个柱子的中心位置添加文本标签和带有圆角的方框
% for i = 1:length(xPositions)
%     x = xPositions(i);
%     y = barHeights(i) / 2;
%     rectangle('Position', [x - boxWidth/2, y - boxHeight/2, boxWidth, boxHeight], ...
%         'EdgeColor', boxColor, 'FaceColor',"#E0E0E0",'Curvature', [0.3 0.3], 'LineWidth', 1);
%     text(x, y, num2str(barHeights(i), '%.2f'), ...
%         'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
% end

% 绘制总成本图
figure;
b3 = bar(sample_result(:,5),'FaceColor',"#FF5722",'EdgeColor',"#8D6E63",'LineWidth',1.2);

set(gca, 'XTickLabel', categories);
ylabel('总成本');
grid on;

% 获取柱状图的X坐标位置
xPositions = b3.XEndPoints;

% 获取柱状图的高度（数据大小）
barHeights = b3.YData;

% 设置方框参数
boxWidth = 0.72; % 方框的宽度
boxHeight = 0.9; % 方框的高度
boxColor = [0.7 0.7 0.7]; % 方框的颜色

% 在每个柱子的中心位置添加文本标签和带有圆角的方框
for i = 1:length(xPositions)
    x = xPositions(i);
    y = barHeights(i) / 2;
    rectangle('Position', [x - boxWidth/2, y - boxHeight/2, boxWidth, boxHeight], ...
        'EdgeColor', boxColor, 'FaceColor',"#E0E0E0",'Curvature', [0.3 0.3], 'LineWidth', 1);
    text(x, y, num2str(barHeights(i), '%.2f'), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
end

%% 红色灵敏度

load KSbase.mat;
KScoef = table2array(readtable('q1coef_poly1.xlsx'));
stimulusData = readtable('./第11题附件/光谱三刺激值加权表.xlsx');
q2targetData = readtable('./第11题附件/样品R值.xlsx');
baseWeight = 2;
lamb = 400:20:700;

% 定义变量上下界
lb = 0 * ones(3, 1);
ub = 0.05 * ones(3, 1);

% 求解器选项
options = optimoptions(@gamultiobj);
options.PopulationSize = 600;
options.ParetoFraction = 0.5;
%options.PlotFcn = "gaplotpareto";

% 变化红色染料价格

add_col = 0;
red_delta_price = [];

for prec = 0.95:0.01:1.05
    curprice = [60 * prec, 65, 63];
    result = {};
    for i = 1:10
        % 计算目标样品的XYZ
        target_XYZ = [0.1 * trapz(lamb, stimulusData.sx .* q2targetData{:, i}), ...
                      0.1 * trapz(lamb, stimulusData.sy .* q2targetData{:, i}), ...
                      0.1 * trapz(lamb, stimulusData.sz .* q2targetData{:, i})];
        % 定义目标函数
        objective = @(vars) [obj(vars, lamb, KScoef, KSbase, stimulusData, target_XYZ); sum(baseWeight .* vars .* curprice)];
        % 定义色差约束
        nonlcon = @(vars) cons(vars, lamb, KScoef, KSbase, stimulusData, target_XYZ);
        
        curResult = [];
        [curResult(:,1:3), curResult(:,4:5)] = gamultiobj(objective, 3, [], [], [], [], lb, ub, nonlcon, options);
        result{i} = sortrows(curResult, 4);
    end

    % 得到对应的价格
    cur_sample_result = [];

    for i = 1:10
        deltaE = result{i}(:,4);
        price = result{i}(:,5);
        [~, index1] = max(deltaE(find(deltaE < quality)));
        cur_sample_result(i,:) = result{i}(index1,:);
    end
    add_col = add_col + 1;
    red_delta_price(:, add_col) = cur_sample_result(:, 5) - sample_result(:, 5);

end


%% 黄色灵敏度

load KSbase.mat;
KScoef = table2array(readtable('q1coef_poly1.xlsx'));
stimulusData = readtable('./第11题附件/光谱三刺激值加权表.xlsx');
q2targetData = readtable('./第11题附件/样品R值.xlsx');
baseWeight = 2;
lamb = 400:20:700;

% 定义变量上下界
lb = 0 * ones(3, 1);
ub = 0.05 * ones(3, 1);

% 求解器选项
options = optimoptions(@gamultiobj);
options.PopulationSize = 600;
options.ParetoFraction = 0.5;
%options.PlotFcn = "gaplotpareto";

% 变化红色染料价格

add_col = 0;
yellow_delta_price = [];

for prec = 0.95:0.01:1.05
    curprice = [60, 65 * prec, 63];
    result = {};
    for i = 1:10
        % 计算目标样品的XYZ
        target_XYZ = [0.1 * trapz(lamb, stimulusData.sx .* q2targetData{:, i}), ...
                      0.1 * trapz(lamb, stimulusData.sy .* q2targetData{:, i}), ...
                      0.1 * trapz(lamb, stimulusData.sz .* q2targetData{:, i})];
        % 定义目标函数
        objective = @(vars) [obj(vars, lamb, KScoef, KSbase, stimulusData, target_XYZ); sum(baseWeight .* vars .* curprice)];
        % 定义色差约束
        nonlcon = @(vars) cons(vars, lamb, KScoef, KSbase, stimulusData, target_XYZ);
        
        curResult = [];
        [curResult(:,1:3), curResult(:,4:5)] = gamultiobj(objective, 3, [], [], [], [], lb, ub, nonlcon, options);
        result{i} = sortrows(curResult, 4);
    end

    % 得到对应的价格
    cur_sample_result = [];

    for i = 1:10
        deltaE = result{i}(:,4);
        price = result{i}(:,5);
        [~, index1] = max(deltaE(find(deltaE < quality)));
        cur_sample_result(i,:) = result{i}(index1,:);
    end
    add_col = add_col + 1;
    yellow_delta_price(:, add_col) = cur_sample_result(:, 5) - sample_result(:, 5);

end

%% 蓝色灵敏度

load KSbase.mat;
KScoef = table2array(readtable('q1coef_poly1.xlsx'));
stimulusData = readtable('./第11题附件/光谱三刺激值加权表.xlsx');
q2targetData = readtable('./第11题附件/样品R值.xlsx');
baseWeight = 2;
lamb = 400:20:700;

% 定义变量上下界
lb = 0 * ones(3, 1);
ub = 0.05 * ones(3, 1);

% 求解器选项
options = optimoptions(@gamultiobj);
options.PopulationSize = 600;
options.ParetoFraction = 0.5;
%options.PlotFcn = "gaplotpareto";

add_col = 0;
blue_delta_price = [];

for prec = 0.95:0.01:1.05
    curprice = [60, 65, 63 * prec];
    result = {};
    for i = 1:10
        % 计算目标样品的XYZ
        target_XYZ = [0.1 * trapz(lamb, stimulusData.sx .* q2targetData{:, i}), ...
                      0.1 * trapz(lamb, stimulusData.sy .* q2targetData{:, i}), ...
                      0.1 * trapz(lamb, stimulusData.sz .* q2targetData{:, i})];
        % 定义目标函数
        objective = @(vars) [obj(vars, lamb, KScoef, KSbase, stimulusData, target_XYZ); sum(baseWeight .* vars .* curprice)];
        % 定义色差约束
        nonlcon = @(vars) cons(vars, lamb, KScoef, KSbase, stimulusData, target_XYZ);
        
        curResult = [];
        [curResult(:,1:3), curResult(:,4:5)] = gamultiobj(objective, 3, [], [], [], [], lb, ub, nonlcon, options);
        result{i} = sortrows(curResult, 4);
    end

    % 得到对应的价格
    cur_sample_result = [];

    for i = 1:10
        deltaE = result{i}(:,4);
        price = result{i}(:,5);
        [~, index1] = max(deltaE(find(deltaE < quality)));
        cur_sample_result(i,:) = result{i}(index1,:);
    end
    add_col = add_col + 1;
    blue_delta_price(:, add_col) = cur_sample_result(:, 5) - sample_result(:, 5);

end

%% 绘制热力图
close all;
load q3red_delta_price.mat
load q3yellow_delta_price.mat
load q3blue_delta_price.mat

categories = {};
for i = 1:10
    categories{i} = "样品" + num2str(i);
end

Xlab = {};
for i = 1:11
    Xlab{i} = num2str(i + 94) + "%";
end

h = heatmap(Xlab,categories,blue_delta_price,'Colormap',jet);


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

