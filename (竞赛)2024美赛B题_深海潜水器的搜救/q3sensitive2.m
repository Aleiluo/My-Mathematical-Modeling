clc,clear;
clear global;
lon_range = [14.6, 22];
lat_range = [34.96, 40.43];
init_data(lon_range, lat_range);
global depth_data;
global rho_data;
global current_data;

%% 在新的位置开始模拟
info.R = 2.2854;
info.r = info.R - 70 * 10^-3; 
info.m = 20 * 10^3;
% 注水量
info.mw = 3.194986632451054e+04;
info.J = 2 * info.m * ((info.R^5 - info.r^5) / (info.R^3 - info.r^3)) / 5;
info.Cd = 0.01;
info.g = 9.8;
info.F = @(t) [0, 0, 113]';

w = 0.005;

info.f_other = @(t) [300*sin(w*t), 300*cos(w*t), 0]';
info.lon_0 = 19.886;
info.lat_0 = 37.148;
info.z0 = -1500;
info.v0 = [-0.02, 0.1, -0.7];
info.t_max = 3 * 3600;
info.time_steps = 0;
info = process_info(info);

DriftOde = @(t, x) q3ode(t, x, info);
options = odeset('Events', @(t,xyz_v) checkOdeBounds(t, xyz_v));
[t, xyz_v] = ode45(DriftOde, info.tspan, info.init_val, options);

% 绘制三维散点图
scatter3(xyz_v(:,1), xyz_v(:,2), xyz_v(:,3));
%%
rows = 60;
cols = 60;
pages = 90;
mu = [xyz_v(:, 1:3), t];
xyzt_bound = [min(mu);max(mu)];
% 边界计算
x = linspace(xyzt_bound(1, 1), xyzt_bound(2, 1), cols);
y = linspace(xyzt_bound(1, 2), xyzt_bound(2, 2), rows);
z = linspace(xyzt_bound(1, 3), xyzt_bound(2, 3), pages);
% 生成网格信息
[X, Y, Z] = meshgrid(x, y, z);
% 将XYZ矩阵整合成n * 3的矩阵
full_grid_points = [reshape(X, [], 1), reshape(Y, [], 1), reshape(Z, [], 1)];


%% 灵敏度分析
% h从1300~4000步长为150

% 估计点到真实点的平均距离
h1_span = 1300:150:4000;
h2acc_mean = [];

for h1 = h1_span
    Pt = zeros(length(t), 4);
    tic;
    parfor i=1:length(t)
        ti = t(i);
        Pt(i, :) = cal_prob(ti, mu, full_grid_points);
    end
    toc
    tmp = mean( Dist(xyz_v(:, 1:3), Pt(:, 1:3)) );
    h2acc_mean = [h2acc_mean, tmp];
    fprintf("h1=%d mean=%f\n",h1, tmp);
end

%% 网格划分

rows = 70;
cols = 70;
pages = 100;
mu = [xyz_v(:, 1:3), t];
xyzt_bound = [min(mu);max(mu)];
% 边界计算
x = linspace(xyzt_bound(1, 1), xyzt_bound(2, 1), cols);
y = linspace(xyzt_bound(1, 2), xyzt_bound(2, 2), rows);
z = linspace(xyzt_bound(1, 3), xyzt_bound(2, 3), pages);
% 生成网格信息
[X, Y, Z] = meshgrid(x, y, z);
% 将XYZ矩阵整合成n * 3的矩阵
full_grid_points = [reshape(X, [], 1), reshape(Y, [], 1), reshape(Z, [], 1)];




%% 
close all;


h1 = 4000;
t_1231 = 1231.90414;
t_len = length(t);
prob_fcn = @(t) cal_full_prob(t, mu, full_grid_points, h1);
P_1231 = prob_fcn(t_1231);
P_1231 = reshape(P_1231, rows, cols, pages);

Pt = zeros(length(t), 4);
parfor i=1:length(t)
    fprintf("i=%d\n",i);
    Pt(i, :) = cal_prob(t(i), mu, full_grid_points, h1);
end


%% 画三维图
init_style();


% 绘制xyz_v的点，指定填充颜色和边界厚度
p1 = scatter3(xyz_v(:,1), xyz_v(:,2), xyz_v(:,3), 'MarkerEdgeColor','#00ACC1',...
         'MarkerFaceColor','#4DD0E1', 'LineWidth',1, 'SizeData', 30);
hold on;
% 绘制Pt的点，这里只选择t_len长度的点，指定填充颜色和边界厚度
p2 = scatter3(Pt(1:t_len,1), Pt(1:t_len,2), Pt(1:t_len,3), 'MarkerEdgeColor','#D81B60',...
         'MarkerFaceColor','#F06292', 'LineWidth',1, 'SizeData', 30);

xlabel('x(m)')
ylabel('y(m)')
zlabel('z(m)')

legend([p1,p2], 'Solutions to Dynamic Equations','Estimated location of Parzen window')

%% 函数

function res = cal_prob(t, mu, full_grid_points, h1)
% parzen窗算法参数
N = size(mu, 1);
V_n = h1 / sqrt(N);
h_n = V_n;

points = size(full_grid_points, 1);
full_grid_points = [full_grid_points, t * ones(points, 1)];
P = zeros(points, 1);
for i = 1:N
    U = (full_grid_points - mu(i, :)) / h_n;
    P = P + mvnpdf(U);
end
% 最后除一个系数
P = P ./ (V_n * N);
% 求出最大的P和对应的坐标
[p, idx] = max(P);
xm = full_grid_points(idx, 1);
ym = full_grid_points(idx, 2);
zm = full_grid_points(idx, 3);
res = [xm, ym, zm, p];

end

function P = cal_full_prob(t, mu, full_grid_points, h1)
N = size(mu, 1);
h1 = 2800;
V_n = h1 / sqrt(N);
h_n = V_n;
points = size(full_grid_points, 1);
full_grid_points = [full_grid_points, t * ones(points, 1)];
P = zeros(points, 1);
for i = 1:N
    U = (full_grid_points - mu(i, :)) / h_n;
    P = P + mvnpdf(U);
end
P = P ./ (V_n * N);
end







