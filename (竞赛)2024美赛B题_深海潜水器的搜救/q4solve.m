clc,clear;
clear global;
lon_range = [-83.241, -59.659];
lat_range = [9.087, 21.162];
q4init_data(lon_range, lat_range);
global depth_data;
global rho_data;
global current_data;

%% 模拟n个潜水器同时出事
close all;
info.R = 2.2854;
info.r = info.R - 70 * 10^-3; 
info.m = 20 * 10^3;
% 注水量
info.mw = 3.18020663e+04;
info.J = 2 * info.m * ((info.R^5 - info.r^5) / (info.R^3 - info.r^3)) / 5;
info.Cd = 0.01;
info.g = 9.8;
info.F = @(t) [0, 0, 0]';
info.f_other = @(t) [0, 0, 0]';
info.lon_0 = inf;
info.lat_0 = inf;
info.z0 = inf;
info.v0 = [];
info.t_max = 10 * 3600;
info.time_steps = 500;
info = process_info(info);

% 确认随机数
rng(44);

% 出事潜水器的个数
n = 6;
center_lon = -65.997;
center_lat = 15.671;
% 继承
full_info = cell(1, n);
for i = 1:n
    full_info{i} = info;
end


% full_info{1}.lon_0 = -67.79915100326407;
% full_info{1}.lat_0 = 14.284350034720172;
% full_info{1}.F = @(t) [0, 0, 40]';
% full_info{1}.time_steps = 1000;
% full_info{2}.lon_0 = -67.80653025731957;
% full_info{2}.lat_0 = 14.278345720486495;
% full_info{2}.F = @(t) [0, 0, 40]';
% full_info{2}.time_steps = 200;
% full_info{3}.lon_0 = -67.79922061886838;
% full_info{3}.lat_0 = 14.272948447428861;
% full_info{3}.F = @(t) [0, 0, 40]';
% full_info{3}.time_steps = 1000;
% full_info{4}.lon_0 = -67.79239828964725;
% full_info{4}.lat_0 = 14.278548115709432;
% full_info{4}.F = @(t) [0, 0, 40]';
% full_info{4}.time_steps = 200;
% full_info{5}.lon_0 = -67.78432287954878;
% full_info{5}.lat_0 = 14.278143325081594;
% full_info{5}.F = @(t) [0, 0, 40]';
% full_info{5}.time_steps = 1000;

t_max = 11111;

for i = 1:n
    % 随机经纬度
    full_info{i}.lon_0 = center_lon + 0.01 * (rand - 0.5);
    full_info{i}.lat_0 = center_lat + 0.01 * (rand - 0.5);
    full_info{i}.F = @(t) [0, 0, 100]';
    full_info{i}.z0 = -1300 * rand;
    full_info{i}.v0 = 0.7 * (rand(1, 3) - 0.5);
    full_info{i} = process_info(full_info{i});

    % 求解
    options = odeset('Events', @(t, xyz_v) checkOdeBounds(t, xyz_v));
    DriftOde = @(t, x) q3ode(t, x, full_info{i});
    [full_info{i}.t, full_info{i}.xyz_v] = ode45(DriftOde, full_info{i}.tspan, full_info{i}.init_val, options);

    % 插值
    full_info{i}.px = spline(full_info{i}.t, full_info{i}.xyz_v(:, 1));
    full_info{i}.py = spline(full_info{i}.t, full_info{i}.xyz_v(:, 2));
    full_info{i}.pz = spline(full_info{i}.t, full_info{i}.xyz_v(:, 3));

    % 求沉底时间
    if full_info{i}.xyz_v(end, 3) < -4000
        full_info{i}.sink_time = full_info{i}.t(end);
    else
        full_info{i}.sink_time = 1e9;
    end

    scatter3(full_info{i}.xyz_v(:,1), full_info{i}.xyz_v(:,2), full_info{i}.xyz_v(:,3));
    hold on;

    t_max = min(t_max, full_info{i}.t(end));
end


%% 问题四主要部分
userConfig.NSALESMEN = 2;
userConfig.XY = zeros(n, 2);
userConfig.MINTOUR = 1;
userConfig.POPSIZE = 15;
userConfig.numIter = 30;
userConfig.fitness = @(route, bp) q4_max_time(route, bp, full_info);
userConfig.SHOWPROG = false;
userConfig.SHOWRESULT = true;
userConfig.SHOWWAITBAR = false;


mtsp = mtsp_ga(userConfig);

%% 解码数据并且绘图

close all;

load q4result.mat

q4_result_fcn = @(route, bp) q4_max_time(route, bp, full_info);
n_rescue = userConfig.NSALESMEN;

% 绘制收敛曲线
init_style();
fitness_history = mtsp.distHistory;
plot(1:length(fitness_history), fitness_history, 'Color','#66BB6A');
xlabel('iterations');
ylabel('t(s)')
legend('Total rescue time');
% ylim([1400, 4000]);

figure();

% 绘制三维搜索图
rescue_points = q4_ans_output(mtsp.optRoute, mtsp.optBreak, full_info);

% 救生艇折线图
colors = ["#26A69A","#8D6E63"];
point_colors = ["#29B6F6","#9CCC65"];
marker_shape = ['^','square'];

array_scale = [0.1,0.08; 0.4,0.2];

for i = 1:n_rescue
    % 绘制线
    plt(i) = plot3(rescue_points{i}(:, 2), rescue_points{i}(:, 3), rescue_points{i}(:, 4), 'LineStyle', '--', ...
        'Color',colors(i));
    hold on;

    % 绘制除了起点和终点之外的散点
    sca(i) = scatter3(rescue_points{i}(2:end-1, 2), rescue_points{i}(2:end-1, 3), rescue_points{i}(2:end-1, 4), ...
        'marker', marker_shape(i), 'MarkerEdgeColor', point_colors(i), ...
        'MarkerFaceColor', point_colors(i), 'LineWidth',1, 'SizeData', 70);

    % 计算第一段线段中点的位置和方向
    startPoint = rescue_points{i}(1, 2:4);
    midPoint1 = (rescue_points{i}(1, 2:4) + rescue_points{i}(2, 2:4)) / 2;
    dir1 = midPoint1 - startPoint;

    % 计算最后一段线段中点的位置和方向
    endPoint = rescue_points{i}(end, 2:4);
    midPoint2 = (rescue_points{i}(end-1, 2:4) + rescue_points{i}(end, 2:4)) / 2;
    dir2 = endPoint - midPoint2;

    % 确定 coneplot 来源点和方向向量
    sources = [midPoint1; midPoint2];
    dirs = [dir1; dir2];

    % Add cones (arrows) using coneplot
    for j = 1:size(sources, 1)
        coneX = sources(j, 1);
        coneY = sources(j, 2);
        coneZ = sources(j, 3);
        velX = dirs(j, 1);
        velY = dirs(j, 2);
        velZ = dirs(j, 3);
        scale = array_scale(i,j);  % Adjust the scaling factor for the cones as needed
        hcone = coneplot([coneX, coneX], [coneY, coneY], [coneZ, coneZ], ...
                 [velX, velX], [velY, velY], [velZ, velZ], ...
                 scale, 'nointerp');
        hcone.FaceColor = colors(i);
        hcone.EdgeColor = 'none';
        hcone.DiffuseStrength = 0.8;
    end
end


for i = 1:n
    t_max = full_info{i}.t(end);
    if t_max > 3000
        t_max = 3000;
    end
    time_span = linspace(0, t_max, 1000); % 创建一个时间范围
    X = ppval(full_info{i}.px, time_span); % 通过参数方程获取X坐标
    Y = ppval(full_info{i}.py, time_span); % 通过参数方程获取Y坐标
    Z = ppval(full_info{i}.pz, time_span); % 通过参数方程获取Z坐标

    % 绘制逐点着色的线段
    surface([X;X],[Y;Y],[Z;Z],[time_span;time_span],...
            'facecol','no',...
            'edgecol','interp',...
            'linew',2); % 使用时间信息进行颜色插值
end
cmap = jet();
colormap(cmap); % 应用之前定义的颜色映射
cb = colorbar;
ylabel(cb, 'Time (s)');
% 设置坐标轴标签和图标题
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');

legend([plt(1),plt(2),sca(1),sca(2)], 'rescue vessel1','rescue vessel2',' rescue point1',' rescue point2');

% % 再绘制海底的地形(灰色网面)
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
% 
% % 边界计算
% 
% x = linspace(xyzt_bound(1, 1)-100, xyzt_bound(2, 1)+100, cols);
% y = linspace(xyzt_bound(1, 2)-100, xyzt_bound(2, 2)+100, rows);
% 
% % 生成网格信息
% [X, Y] = meshgrid(x, y);


%%
[x,y,z] = meshgrid(-1:.1:1,-1:.1:1,-3:.1:3);
[u,v,w] = deal(y./sqrt(x.^2+y.^2),-x./sqrt(x.^2+y.^2),1./(cos(z/3)+1));
c = num2cell((rand(2000,3)-.5)*diag([2 2 6]),1);
set(coneplot(x,y,z,u,v,w,c{:},2),'facec',[.2 .7 1],'edgec','n')
axis image, view(3), camlight, lighting gouraud


%% 

function points = q4_ans_output(routes, bps, full_info)
    % 输入:
    % route: 总路线
    % bp: 分割点
    % full_info: 所有求解信息

    % 返回每一个救生舱的关键位置信息
    points = {};

    % 所有救生艇的速度
    fitness = -32767;
    v_dive = -2.056;
    
    % 解析每一段路径，计算每一个救生舱运行状态
    bps = [0, bps, length(routes)];
    for i = 1:(length(bps) - 1)
        % 第i个救生艇的行驶路线
        cur_route = routes(bps(i) + 1:bps(i + 1));
        % 求解下潜方程vt=z(t)得到见面时间
        f = @(t) t .* v_dive - ppval(full_info{cur_route(1)}.pz, t);
        cur_t = fsolve(f, 0, optimset('Display','off'));
        cur_x = ppval(full_info{cur_route(1)}.px, cur_t);
        cur_y = ppval(full_info{cur_route(1)}.py, cur_t);
        cur_z = ppval(full_info{cur_route(1)}.pz, cur_t);
        points{i} = [];
        points{i} = [points{i}; [0, cur_x, cur_y, 0]];
        points{i} = [points{i}; [cur_t, cur_x, cur_y, cur_z]];

        % 继续求该路径上的其它点
        for j = 2:length(cur_route)
            f = @(t) (v_dive * t).^2 - ...
                (ppval(full_info{cur_route(j)}.px, cur_t + t) - cur_x).^2 - ...
                (ppval(full_info{cur_route(j)}.py, cur_t + t) - cur_y).^2 - ...
                (ppval(full_info{cur_route(j)}.pz, cur_t + t) - cur_z).^2;
            t2next = fsolve(f, cur_t + 1000, optimset('Display','off'));
            % 如果救援时间超过了沉底时间，t2next无解，返回一个大值
            if t2next < 0
                fitness = 3e5;
                return
            end

            cur_t = cur_t + t2next;
            cur_x = ppval(full_info{cur_route(j)}.px, cur_t);
            cur_y = ppval(full_info{cur_route(j)}.py, cur_t);
            cur_z = ppval(full_info{cur_route(j)}.pz, cur_t);
            points{i} = [points{i}; [cur_t, cur_x, cur_y, cur_z]];
%             % 如果救援时间超过了沉底时间，返回一个大值
%             if cur_t > full_info{cur_route(j)}.sink_time
%                 fitness = 1e6;
%                 return
%             end
        end
        % 最后带着它的老婆孩子浮到水面
        points{i} = [points{i}; [cur_t + points{i}(end, 4) / v_dive, ...
            points{i}(end, 2), points{i}(end, 3), 0]];

        % 救完这一条路线后
        fitness = max(fitness, cur_t);
    end
end




