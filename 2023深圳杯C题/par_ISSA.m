function [Xbest, Fbest, gen_x, gen_fitness] = par_ISSA(fobj, x0, dim, lb, ub, popSize, maxIter, usingParallel)
% 并行化改进麻雀算法实现
% 假设fobj有一个主循环，应该给这个循环一个尽量小的精确的循环次数
% 这样并行的效率才快

%% DEBUG
% rng(4);

%% 输入检查
if nargin < 8
    usingParallel = false;
end

%% 数据检查

if length(lb) == 1 && length(ub) == 1
    lb = lb .* ones(1, dim);
    ub = ub .* ones(1, dim);
elseif length(lb) ~= dim || length(ub) ~= dim
    error('上下界维数错误');
end

%% 参数设置

% --- SSA参数 ---
% 探索者比例
P_finder = 0.4;
% 意识到危险个体的比例
P_warning = 0.2;
% 安全阈值
ST = 0.8;

if usingParallel
    userInput = input('警惕者适应度更新无法并行化，是否将警惕者比例设为0? (y/n): ', 's');
    if strcmpi(userInput, 'y')
        P_warning = 0;
    end
end

% --- 其他参数 ---

% 每代最优值
gen_fitness = zeros(maxIter, 1);
% 每代最优个体
gen_x = zeros(maxIter, dim);
% 探索者个体数
finderPop = floor(popSize * P_finder);
% 预警个体数
alertPop = floor(popSize * P_warning);
% 判断等于误差
Eps = 1e-6;


%% 种群初始化

% % 佳点集初始化
% x = lb + (ub - lb) .* Goodnode(popSize, dim);

%随机初始化
x = lb + (ub - lb) .* rand(popSize, dim);

% 若传入x0就插入
if ~isempty(x0)
    x(1, :) = x0;
end

% 计算适应度
fitness = zeros(popSize, 1);
if usingParallel == true
    parfor i = 1:popSize
        fitness(i) = fobj(x(i, :));
    end
else
    for i = 1:popSize
        fitness(i) = fobj(x(i, :));
    end
end

% 循环外进行第一趟排序
[fitness, sortIdx] = sort(fitness);
x = x(sortIdx, :);

x_best = x;
fitness_best = fitness;

% 最优个体
Xbest = x_best(1, :);
Fbest = fitness_best(1);
% 最劣个体
Xworst = x_best(end, :);
Fworst = fitness_best(end);


%% 主循环
for it = 1:maxIter
    %% 探索者位置更新
    R2 = rand;
    if R2 < ST
        % 批量更新x
        x(1:finderPop, :) = x_best(1:finderPop, :) .* exp(-(1:finderPop)' ./ (rand(finderPop, 1) .* maxIter));
        % 批量修复上下界
        x(1:finderPop, :) = BoundFix(x(1:finderPop, :), lb, ub);
        % 循环计算目标值
        if usingParallel == true
            parfor i = 1:finderPop
                fitness(i) = fobj(x(i, :));
            end
        else
            for i = 1:finderPop
                fitness(i) = fobj(x(i, :));
            end
        end
    else
        % 方法同上
        x(1:finderPop, :) = x_best(1:finderPop, :) + randn(finderPop, 1) .* ones(finderPop, dim);
        x(1:finderPop, :) = BoundFix(x(1:finderPop, :), lb, ub);
        if usingParallel == true
            parfor i = 1:finderPop
                fitness(i) = fobj(x(i, :));
            end
        else
            for i = 1:finderPop
                fitness(i) = fobj(x(i, :));
            end
        end
    end

    %% 追随者位置更新

    % 计算该代最优位置Xp
    [~, Xp_idx] = min(fitness);      
    Xp = x(Xp_idx, :);         

    i1 = floor(popSize / 2) + 1:popSize;
    % 性能较差的个体使用最劣个体信息更新
    x(i1, :) = randn(length(i1), 1) .* exp((Xworst - x_best(i1, :)) ./ i1.^2');
    % 性能较优的个体使用最优个体信息更新
    for i = finderPop + 1:floor(popSize / 2)
        A = floor(rand(1, dim) * 2) * 2 - 1;
        Aplus = A' * (A * A').^(-1);
        x(i, :) = Xp + abs(x_best(i, :) - Xp) * Aplus * ones(1, dim);  
    end
    % 更新后修复边界
    x(finderPop + 1:popSize, :) = BoundFix(x(finderPop + 1:popSize, :), lb, ub);
    % 计算追随者的目标值
    if usingParallel == true
        parfor i = finderPop + 1:popSize
            fitness(i) = fobj(x(i, :));
        end
    else
        for i = finderPop + 1:popSize
            fitness(i) = fobj(x(i, :));
        end
    end

    %% 警觉者位置更新

    % 警觉者选取
    alertIdx = randperm(popSize);
    alertIdx = alertIdx(1:alertPop);
    for i = alertIdx
        if fitness_best(i) > Fbest
            % 使用全局最优个体更新
            x(i, :) = Xbest +  randn(1,dim) .* abs(x_best(i, :) - Xbest);
        elseif abs(fitness_best(i) - Fbest) < Eps
            % 等于最优值
            x(i, :) = x_best(i, :) + (2 * rand - 1) .* ...
                abs(x_best(i, :) - Xworst) / (fitness_best(i) - Fworst + eps);
        end
            % 其它情况不进行更新
    end

    % 修复边界并更新目标函数
    x(alertIdx, :) = BoundFix(x(alertIdx, :), lb, ub);
    % 计算追随者的目标值(警觉者这边不适合使用并行)
    for i = alertIdx
        fitness(i) = fobj(x(i, :));
    end


    %% 一轮位置更新结束

    % 第一步：更新历史最优值
    updateIdx = fitness < fitness_best;
    fitness_best(updateIdx) = fitness(updateIdx);
    x_best(updateIdx, :) = x(updateIdx, :);

    % 第二步：历史最优值排序
    [fitness_best, sortIdx] = sort(fitness_best);
    x_best = x_best(sortIdx, :);

    % 第三步：将当前种群与历史最优值对齐
    x = x(sortIdx, :);
    fitness = fitness(sortIdx, :);
    
    % 第四步：选择最优、最劣个体
    Xbest = x_best(1, :);
    Fbest = fitness_best(1);
    Xworst = x_best(end, :);
    Fworst = fitness_best(end);

    % 记录最优值
    gen_fitness(it) = Fbest;
    gen_x(it, :) = Xbest;
    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(Fbest)]);
end

end

function x = BoundFix(x, lb, ub)
    % 批量修复上下界
    xlen = size(x, 1);
    x = max(min(x, repmat(ub, xlen, 1)), repmat(lb, xlen, 1));
end

function x = Goodnode(M, N)
    % 构建佳点集
    Prime = primes(100 * N);
    idx = find(Prime >= (2 * N + 3), 1);
    x = ((1:M)' * ones(1, N)) .* (2 * cos((2 * pi * (1:N)) / Prime(idx)));
    x = mod(x, 1);
end