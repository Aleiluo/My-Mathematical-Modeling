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

% cal_prob(0, mu, full_grid_points)


Pt = zeros(length(t), 4);
tic;
parfor i=1:length(t)
    ti = t(i);
    fprintf("%d\n",ti);
    Pt(i, :) = cal_prob(ti, mu, full_grid_points);
end
toc



% 使用三次样条插值
max_t = t(end);
t_len = length(t);
px = spline(t, Pt(:, 1));
py = spline(t, Pt(:, 2));
pz = spline(t, Pt(:, 3));

t_span = linspace(0, max_t, 1000);
scatter3(xyz_v(:,1), xyz_v(:,2), xyz_v(:,3));
hold on;
plot3(ppval(px, t_span), ppval(py, t_span), ppval(pz, t_span));
scatter3(Pt(1:t_len,1), Pt(1:t_len,2), Pt(1:t_len,3));

scatter3(Pt(31,1), Pt(31,2), Pt(31,3), marker='*');

%% 求解见面方程t0*v - z(t0) = 0 
% 下潜速度：4节满速
v_dive = -2.056;
f = @(t) t .* v_dive - ppval(pz, t);
plot(t_span, f(t_span));

t_meet = fsolve(f,0);

% 得到下潜坐标
x_dive = ppval(px, t_meet);
y_dive = ppval(py, t_meet);

%% 下潜的过程中，计算发现潜水器的概率
% 重新计算样本点更多的ode

R = 100;
z_dive = xyz_v(1, 3) + R;
% 下潜深度何时超过事发深度
t_start = z_dive / v_dive;
% 每2.5秒求一次搜寻的概率
dt = 2.5;
prob_recoder = [];
% 创建快捷函数
prob_fcn = @(t) cal_full_prob(t, mu, full_grid_points);

for ti = t_start:dt:t_meet
    % 求当前时刻三维空间概率
    P = cal_full_prob(ti, mu, full_grid_points);
    % 概率积分
    p_tmp = prob_integral(x_dive, y_dive, z_dive, full_grid_points, P, R);
    prob_recoder = [prob_recoder; [ti, z_dive, p_tmp]];

    fprintf("%f %f %f\n",z_dive, ti, p_tmp);
    % 向下运动
    z_dive = z_dive + dt * v_dive;
    
end

%% 数据导出

% P_req, full_grid_points, xyz_t, t, Pt
% t = 1231.90414s 时候输出位置网格和概率网格
t_req = 500;
P_1231 = prob_fcn(1231.90414);
P_1231 = reshape(P_1231, rows, cols, pages);
% t_span, 插值后的数据
inter_time = t_span';
inter_data = [ppval(px, t_span)', ppval(py, t_span)', ppval(pz, t_span)'];

% prob_recoder

save('data2py/q3_data.mat', "X","Y","Z", "P_1231", "t", "xyz_v", "Pt", ...
    "inter_time", "inter_data", "prob_recoder")

%% 画三维图
close all;
init_style();
xslice = [2.214268311182934e+06];
yslice = [4.459756204506748e+06];
zslice = [];

% 创建切片
s = slice(X, Y, Z, P_1231, xslice, yslice, zslice);
colormap(hot);
% 遍历返回的切片对象，设置边缘无颜色
for i = 1:length(s)
    s(i).EdgeColor = 'none';
end

hold on;

% 绘制xyz_v的点，指定填充颜色和边界厚度
p1 = scatter3(xyz_v(:,1), xyz_v(:,2), xyz_v(:,3), 'MarkerEdgeColor','#00ACC1',...
         'MarkerFaceColor','#4DD0E1', 'LineWidth',1, 'SizeData', 30);

% 绘制轨迹（如果已定义px, py, pz 和 t_span）
p2 = plot3(ppval(px, t_span), ppval(py, t_span), ppval(pz, t_span), 'Color', '#FFEE58');

% 绘制Pt的点，这里只选择t_len长度的点，指定填充颜色和边界厚度
p3 = scatter3(Pt(1:t_len,1), Pt(1:t_len,2), Pt(1:t_len,3), 'MarkerEdgeColor','#D81B60',...
         'MarkerFaceColor','#F06292', 'LineWidth',1, 'SizeData', 30);

p4 = scatter3(Pt(31,1), Pt(31,2), Pt(31,3), 'marker','diamond', 'MarkerEdgeColor','#7C4DFF',...
         'MarkerFaceColor','#7C4DFF', 'LineWidth',1, 'SizeData', 70);

xlabel('x(m)')
ylabel('y(m)')
zlabel('z(m)')

legend([p1,p3,p2,p4], 'Solutions to Dynamic Equations','Estimated location of Parzen window',...
    'Interpolation Trajectory','The maximum position of probability density when t=1231.9s')

%% 函数

function res = cal_prob(t, mu, full_grid_points)
% parzen窗算法参数
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
% 最后除一个系数
P = P ./ (V_n * N);
% 求出最大的P和对应的坐标
[p, idx] = max(P);
xm = full_grid_points(idx, 1);
ym = full_grid_points(idx, 2);
zm = full_grid_points(idx, 3);
res = [xm, ym, zm, p];

end

function prob = prob_integral(x, y, z, points, P, R)
    Idx = (x - points(:, 1)).^2 + (y - points(:, 2)).^2 + (z - points(:, 3)).^2 < R^2;
    prob = sum(P(Idx));
end

function P = cal_full_prob(t, mu, full_grid_points)
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






