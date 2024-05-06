clc,clear;
clear global;
lon_range = [14.6, 22];
lat_range = [34.96, 40.43];
init_data(lon_range, lat_range);
global depth_data;
global rho_data;
global current_data;

%% 绘制两个
Tx = readtable('data/q1sensitive沉底.xlsx','Sheet','x_rand');
Ty = readtable('data/q1sensitive沉底.xlsx','Sheet','y_rand');
Tz = readtable('data/q1sensitive沉底.xlsx','Sheet','z_rand');
Tori = readtable('data/q1sensitive沉底.xlsx','Sheet','xyz_origin');
Dx = table2array(Tx);
Dy = table2array(Ty);
Dz = table2array(Tz);
Dori = table2array(Tori);



%%
close all;
% 绘制主轨迹
init_style()
p1 = plot3(Dori(:, 1), Dori(:, 2), Dori(:, 3), Color="#29B6F6");
hold on;
s1 = scatter3(Dori(end, 1), Dori(end, 2), Dori(end, 3), 'SizeData',70, 'MarkerEdgeColor','#EF5350'...
    , 'MarkerFaceColor','#EF5350', 'Marker','pentagram');

for i = 1:size(Dx, 2)
    p = plot3(Dx(:, i), Dy(:, i), Dz(:, i),'Color', '#BDBDBD','LineStyle','--');
    p.Color(4) = 0.6;
end

last_points = [Dx(end, :)', Dy(end, :)', Dz(end, :)'];

s2 = scatter3(last_points(:,1), last_points(:,2), last_points(:,3), 'SizeData',10, 'MarkerEdgeColor','#FFA726'...
    , 'MarkerFaceColor','#FFEE58');

legend([p1,s1,s2],'solutions to equations of motion','sinking position','offset sunken position')
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
% 选择一个局部区域进行放大
% 这里要做局部放大，你需要定义放大的范围，这是一个范例：
xlim_local = [min(last_points(:, 1)), max(last_points(:, 1))]; % x轴放大范围
ylim_local = [min(last_points(:, 2)), max(last_points(:, 2))]; % y轴放大范围
zlim_local = [min(last_points(:, 3)), max(last_points(:, 3))]; % z轴放大范围

% 在原图上插入一个新的小图像
axes('Position', [.25 .3 .3 .3]); % 设置小图位置和大小
box on; % 显示小图的边框

% 在放大的轴上绘制相同的数据
plot3(Dori(:, 1), Dori(:, 2), Dori(:, 3), Color="#29B6F6");
hold on;


for i = 1:size(Dx, 2)
    p = plot3(Dx(:, i), Dy(:, i), Dz(:, i), 'Color', rgbaColor, 'LineStyle', '--');
    p.Color(4) = 0.6;
end
scatter3(last_points(:,1), last_points(:,2), last_points(:,3), 'SizeData',10, 'MarkerEdgeColor','#FFA726'...
    , 'MarkerFaceColor','#FFEE58');
scatter3(Dori(end, 1), Dori(end, 2), Dori(end, 3), 'SizeData',70, 'MarkerEdgeColor','#EF5350'...
    , 'MarkerFaceColor','#EF5350', 'Marker','pentagram');
% 利用函数xlim, ylim, 和 zlim 设置放大轴的显示范围
xlim(xlim_local);
ylim(ylim_local);
zlim(zlim_local);

% % 计算所有最后点的Z值的平均高度
% avg_height = mean(last_points(:,3));
% 
% % 设置网格点进行KDE
% xmin = min(last_points(:, 1));
% xmax = max(last_points(:, 1));
% ymin = min(last_points(:, 2));
% ymax = max(last_points(:, 2));
% gridX = linspace(xmin, xmax, 100);
% gridY = linspace(ymin, ymax, 100);
% [gridX, gridY] = meshgrid(gridX, gridY);
% 
% % 执行核密度估计
% [bandwidth, density, X, Y] = kde2d(last_points(:, 1:2), 256, [xmin ymin], [xmax ymax]);
% 
% % 将密度归一化
% density = density / max(density(:));
% 
% % 绘制平均高度平面上的热力图
% Zgrid = avg_height * ones(size(X)); % 创建Z值为平均高度的网格
% surf(X, Y, Zgrid, density, 'EdgeColor', 'none');
% colormap(jet); % 使用Jet颜色映射
% colorbar; % 显示颜色条
% 
% % 设置视图和轴的属性
% view(3); % 切换到三维视图


