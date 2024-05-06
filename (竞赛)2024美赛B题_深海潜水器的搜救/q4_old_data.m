% % 网格划分
% mu = [full_info{1}.xyz_v(:, 1:3), full_info{1}.t];
% xyzt_bound = [min(mu); max(mu)];
% for i = 2:5
%     mu = [full_info{i}.xyz_v(:, 1:3), full_info{i}.t];
%     xyzt_bound(1, :) = min(xyzt_bound(1, :), min(mu));
%     xyzt_bound(2, :) = max(xyzt_bound(2, :), max(mu));
% end
% rows = 70;
% cols = 70;
% pages = 100;
% % 边界计算
% x = linspace(xyzt_bound(1, 1), xyzt_bound(2, 1), cols);
% y = linspace(xyzt_bound(1, 2), xyzt_bound(2, 2), rows);
% z = linspace(xyzt_bound(1, 3), xyzt_bound(2, 3), pages);
% % 生成网格信息
% [X, Y, Z] = meshgrid(x, y, z);
% % 将XYZ矩阵整合成n * 3的矩阵
% full_grid_points = [reshape(X, [], 1), reshape(Y, [], 1), reshape(Z, [], 1)];
% 
% for i = 1:5
%     t_len = length(full_info{i}.t);
%     if t_len > 70
%         delta_idx = floor(t_len / 70);
%         full_info{i}.t = full_info{i}.t(1:delta_idx:t_len);
%         full_info{i}.xyz_v = full_info{i}.xyz_v(1:delta_idx:t_len, :);
%     end
%     
%     Pt = zeros(length(full_info{i}.t), 4);
%     mu = [full_info{i}.xyz_v(:, 1:3), full_info{i}.t];
%     t_tmp = full_info{i}.t;
%     
%     tic;
%     parfor j = 1:length(t_tmp)
%         ti = t_tmp(j);
%         fprintf("%d\n",ti);
%         Pt(j, :) = cal_prob(ti, mu, full_grid_points);
%     end
%     toc
% 
%     full_info{i}.Pt = Pt;
% 
%     scatter3(full_info{i}.xyz_v(:,1), full_info{i}.xyz_v(:,2), full_info{i}.xyz_v(:,3));
%     hold on;
%     scatter3(full_info{i}.Pt(:,1), full_info{i}.Pt(:,2), full_info{i}.Pt(:,3));
% end



%% 注水量估计

std_v = -1;

% 求下潜需要额外注水的重量mw

i = 3;
full_info{i}.z0 = -10;
mw = cal_mw(std_v, full_info{i});

DriftOde = @(t, x) q3ode(t, x, full_info{i});
[full_info{i}.t, full_info{i}.xyz_v] = ode45(DriftOde, full_info{i}.tspan, full_info{i}.init_val, options);

scatter3(full_info{2}.xyz_v(:,1), full_info{2}.xyz_v(:,2), full_info{2}.xyz_v(:,3));
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
