function [loss,uu] = ImpTempLoss(h, dx, dt, ClothInfo, insTemp)
% 使用隐式计算温度矩阵
%
% h：牛顿冷却公式中热交换系数，该模型有两个热交换系数：h(1)、h(2)
% dx,dt：长度、时间分割步长
% ClothInfo：防护服参数，1~4行为服装的1~4层。1~4列分别是：材料密度、比热、热传导率、厚度
% insTemp：实测的体表温度数据

%% 确认方程参数
bodyTemp = 37;  % 人体温度
envTemp = 75;   % 环境温度
rho = ClothInfo(:, 1);
cap = ClothInfo(:, 2);
lam = ClothInfo(:, 3);
s = (lam .* dt) ./ (rho .* cap .* dx'.^2);

tlen = ceil(length(insTemp)/dt); % 总共划分的时间
nx = zeros(4, 2); % 四个材料区间的左右端点
for i = 1:4
    tmplen = ceil(ClothInfo(i, 4)/dx(i));
    if i == 1
        nx(i, 1) = 1;
        nx(i, 2) = tmplen;
    else
        nx(i, 1) = nx(i - 1, 2);
        nx(i, 2) = nx(i - 1, 2) + tmplen;
    end
end

%% 构建各矩阵

% 温度分布矩阵
xlen = nx(4,2);
u = zeros(xlen, tlen);

% 系数矩阵中的参数
a = zeros(xlen, 1);
b = zeros(xlen, 1);
c = zeros(xlen, 1);

% 控制方程
for k = 1:4
    l = nx(k,1);        % 取区间的左右端点
    r = nx(k,2);
    a(l + 1:r - 1) = -s(k);
    b(l + 1:r - 1) = 1 + 2 * s(k);
    c(l + 1:r - 1) = -s(k);
end

% 接触面
for k = 1:3
    r = nx(k, 2);      % 区间的右端点就是交界处
    a(r) = -lam(k) * dx(k+1);
    b(r) = lam(k) * dx(k+1) + lam(k+1) * dx(k);
    c(r) = -lam(k+1) * dx(k);
end

% 边界条件
b(1) = dx(1) * h(1) + lam(1);
c(1) = -lam(1);
a(end) = -lam(4);
b(end) = dx(4) * h(2) + lam(4);

% 初始条件
u(:,1) = bodyTemp;

%% 追赶法求解

% 第一步：计算beta
beta = zeros(xlen, 1);
beta(1) = c(1) / b(1);
for i = 2:xlen
    beta(i) = c(i) / (b(i) - a(i) * beta(i - 1));
end

% 第二步：不断重复解Ly = f和解Ux = y可以得到温度分布矩阵

f = zeros(xlen, 1);
y = zeros(xlen, 1);
for j = 2:tlen
    % 构造f
    f(1) = dx(1) * h(1) * envTemp;           % 左边界
    f(end) = dx(4) * h(2) * bodyTemp;        % 右边界
    f(nx([1, 2, 3], 2)) = 0;                 % 交界处
    for k = 1:4
        l = nx(k, 1);        % 取区间的左右端点
        r = nx(k, 2);
        f(l + 1:r - 1) = u(l + 1:r - 1, j - 1);
    end

    % 解Ly = f
    y(1) = f(1) / b(1);
    for i = 2:xlen
        y(i) = (f(i) - a(i) * y(i - 1)) / (b(i) - a(i) * beta(i - 1));
    end

    % 解Ux = y，这里的x相当于当前时刻u的列
    u(end, j) = y(end);
    for i = xlen-1:-1:1
        u(i, j) = y(i) - beta(i) * u(i + 1, j);
    end
end

%% 输出

tper = ceil(1/dt); % 计算1s相当于网格中的多少格

% 提取u中的最后一行，并且做整秒的映射
uu = u(:, 1:tper:tlen);

% 计算loss
if size(insTemp, 1) > 1, insTemp = insTemp'; end
loss = sum((uu(end,:) - insTemp).^2) / length(insTemp);

end