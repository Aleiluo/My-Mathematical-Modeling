function [loss,uu] = ExpTempLoss(h, dx, dt, ClothInfo, insTemp)
% 使用显式计算温度矩阵
%
% h：牛顿冷却公式中热交换系数，该模型有两个热交换系数：h(1)、h(2)
% dx,dt：长度、时间分割步长
% ClothInfo：防护服参数，1~4行为服装的1~4层。1~4列分别是：材料密度、比热、热传导率、厚度
% insTemp：实测的体表温度数据

%% 确认方程参数
bodyTemp = 37; % 人体温度
envTemp = 75; % 环境温度
rho = ClothInfo(:, 1);
c = ClothInfo(:, 2);
lam = ClothInfo(:, 3);
s = (lam .* dt) ./ (rho .* c .* dx'.^2);

nt = ceil(length(insTemp)/dt); % 总共划分的时间
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

%% 有限元差分

% 确认热力矩阵大小以及初值

u = zeros(nx(4, 2), nt);
u(:, 1) = bodyTemp; % 初始温度设为人体温度

for j = 2:nt
    % 使用控制方程进行递推
    for k = 1:4
        l = nx(k, 1);
        r = nx(k, 2);
        u(l+1:r-1, j) = (1 - 2 * s(k)) .* u(l+1:r-1, j-1) + s(k) .* (u(l:r-2, j-1) + u(l+2:r, j-1));
    end
    % 对于各层之间的接触面：值和导数都要相等，即可推出接触面的那两个点的温度
    for k = 1:3
        curR = nx(k, 2); % 表示当前区间的右边界
        u(curR, j) = (lam(k) * u(curR-1, j) + lam(k+1) * u(curR+1, j)) / (lam(k) + lam(k+1));
    end
    % 对于外接与第一层接触面，人体与第四层接触面，均满足牛顿冷却公式

    % 使用高级版牛顿冷却公式(实际上是别人论文的东西，怎么推导不知道)
    % u(1, j) = (-2 * h(1) * dt) / (dx(1) * rho(1) * c(1)) * (u(1, j-1) - envTemp) - 2 * s(1) * (u(1,j-1) - u(2,j-1)) + u(1,j - 1);
    % u(end, j) = (-2 * h(2) * dt) / (dx(4) *rho(4) * c(4)) * (u(end, j-1) - bodyTemp) + 2 * s(4) * (u(end - 1, j-1) - u(end, j-1)) + u(end, j - 1);

    % 使用原始牛顿冷却公式
    %左边界原本应该使用这个公式，但是不知道为什么错了
    %u(1, j) = (lam(1) * u(2, j) - dx(1) * h(1) * envTemp) / (lam(1) - dx(1) * h(1));

    u(1, j) = (lam(1) * u(2, j) + dx(1) * h(1) * envTemp) / (lam(1) + dx(1) * h(1));
    u(end, j) = (lam(4) * u(end-1, j) + dx(4) * h(2) * bodyTemp) / (lam(4) + dx(4) * h(2));
end

%% 输出

tper = ceil(1/dt); % 计算1s相当于网格中的多少格

% 提取u中的最后一行，并且做整秒的映射
uu = u(:, 1:tper:nt);

% 计算loss
if size(insTemp, 1) > 1, insTemp = insTemp'; end
loss = sum((uu(end,:) - insTemp).^2) / length(insTemp);

end