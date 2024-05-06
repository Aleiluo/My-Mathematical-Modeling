clc,clear;

% 读取坐标
load 测试坐标.mat
mirror_number = size(mirrors, 1);
% scatter(T1.x, T1.y)
% axis equal

%% 参数设置

% --- 时间参数 ---
q1_day_number = 12;
q1_times_number = 5;
% 12个月，5个时间点
q1_days = datetime(2023, 1:q1_day_number, 21);
% 一天5个时间点
q1_daytimes = [9, 10.5, 12, 13.5, 15];
% 春分日期
SpringEquinox = datetime(2023, 3, 21);

% --- 反射镜与接收塔参数 ---
param.len = 6;
param.width = 6;
param.height = 4;
param.Hz = 80;
% 集热器高度
param.col_h = 8;
param.col_d = 7;
% 假设定日镜距离大于一定值，两者必不可能发生遮蔽现象
param.max_dis = 30;
param.eta_ref = 0.92;

% --- 遮蔽损失参数 ---
subs = 12;
sub_rad = 2 * pi / subs;
full_info_cnt = 0;
full_info = {};
sub_mirrors = {};

% sub_mirrors各列信息介绍：
% 1:镜子中心x
% 2:镜子中心y
% 3:镜子中心z
% 4:镜子长度
% 5:镜子宽度
% 6:镜子到(0,0)的距离
% 7:俯仰角
% 8:方位角
% 
% 9 nx
% 10 ny
% 11 nz
% 
% 12 ~ 26：5点坐标
% 
% 27:遮蔽效率
% 28:集热器截断效率
% 29:余弦效率
% 30:大气透射效率
% 31:镜面反射效率
% 32:光学效率
% 33:Ai * eta_i



%% 问题求解

% 初始化镜面信息
for i = 1:subs
    sub_mirrors{i} = [];
end
% 坐标划分
for i = 1:mirror_number
    % 使用四象限反正切计算坐标夹角
    pos = ceil((atan2(mirrors(i, 2), mirrors(i, 1)) + pi) / sub_rad);
    sub_mirrors{pos} = [sub_mirrors{pos}; ...
        [mirrors(i, 1), mirrors(i, 2), param.height, param.len, param.width, sqrt(mirrors(i, 1)^2 + mirrors(i, 2)^2)]];
end
% 对立面点进行距离排序
for i = 1:subs
    sub_mirrors{i} = sortrows(sub_mirrors{i}, 6, "descend");
end

k = 0;
q1_res = zeros(q1_day_number, 7);
for i = 1:q1_day_number
    dayRes_mean = zeros(5, 7);
    D = days(q1_days(i) - SpringEquinox);
    for j = 1:q1_times_number
        ST = q1_daytimes(j);
        disp(q1_days(i));
        disp(ST);
        % 求此刻的DNI、太阳的高度角、方位角
        param = calDNI(D, ST , param);

        % 求此刻所有镜子的方位角和俯仰角
        sub_mirrors = cal_mirror_info(sub_mirrors, param);

        % 计算各个指标
        sub_mirrors = cal_target(sub_mirrors, param);
        
        % 记录信息
        full_info_cnt = full_info_cnt + 1;
        full_info{full_info_cnt} = sub_mirrors;

        % 求均值
        timeRes_mean = zeros(1, 7);
        tot_area = 0;
        for k = 1:subs
            % 第一列：光学效率
            timeRes_mean(1) = timeRes_mean(1) + sum(sub_mirrors{k}(:, 32));
            % 第二列：余弦效率
            timeRes_mean(2) = timeRes_mean(2) + sum(sub_mirrors{k}(:, 29));
            % 第三列：遮挡效率
            timeRes_mean(3) = timeRes_mean(3) + sum(sub_mirrors{k}(:, 27));
            % 第四列：截断效率
            timeRes_mean(4) = timeRes_mean(4) + sum(sub_mirrors{k}(:, 28));
            % 第五列：大气效率
            timeRes_mean(5) = timeRes_mean(5) + sum(sub_mirrors{k}(:, 30));
            % 第六列：总功率
            timeRes_mean(6) = timeRes_mean(6) + param.DNI * sum(sub_mirrors{k}(:, 33));
            % 总面积
            tot_area = tot_area + sum(sub_mirrors{k}(:, 4) .* sub_mirrors{k}(:, 5));
        end
        % 计算一时刻的平均效率
        timeRes_mean(1:5) = timeRes_mean(1:5) / mirror_number;
        % 第七列：单位面积功率
        timeRes_mean(7) = timeRes_mean(6) / tot_area;
        
        dayRes_mean(j, :) = timeRes_mean;
    end
    q1_res(i, :) = mean(dayRes_mean, 1);
end

full_info = reshape(full_info, q1_times_number, 12);

% %% 结果输出
% close all;
% 
% % 绘制1月21日5个时间点的镜片利用率、遮挡率
% for i = 1:5
%     fig = figure();
%     A = cat2one(full_info{i});
%     % 绘制每一片镜子的总效率
%     scatter(A(:,1), A(:,2), 20, A(:,32), 'filled');
%     axis equal;
%     axis off;
%     ax = gca;
%     set(ax, 'Color', 'none');
%     set(fig, 'Color', 'none');
%     % 保存当前图为 PNG 图像
%     filename = "1月21日时间点" + num2str(i) + ".png";
%     print(gcf, filename, '-dpng', ['-r' num2str(400)]);
% end
% %% 夏天正午图
% close all;
% k = 6;
% for i = (k - 1) * 5 + 3
%     A = cat2one(full_info{i});
%     % 绘制每一片镜子的总效率
%     figure;
%     scatter(A(:,1), A(:,2), 20, A(:,32), 'filled');
%     colorbar;
%     axis equal;
%     saveas(gcf, "q1-6月21日正午.svg");
% end
% %% 绘制各个效率的占比曲线图(横坐标为月份)
% close all;
% plot(q1_res(:,1), LineWidth=1.8, color='#F44336', Marker='pentagram'); hold on;
% plot(q1_res(:,2), LineWidth=1.8, color='#FFEE58',LineStyle='-.', Marker='*');
% plot(q1_res(:,3), LineWidth=1.8, color='#66BB6A',LineStyle='-.', Marker='x');
% plot(q1_res(:,4), LineWidth=1.8, color='#D4E157',LineStyle='-.', Marker='^');
% plot(q1_res(:,5), LineWidth=1.8, color='#EC407A',LineStyle='-.', Marker='square');
% ylim([0.5, 1]);
% ylabel('效率');
% xlabel('月份');
% xticks(1:12);
% legend('日均光学效率','日均余弦效率','日均遮挡效率','日均截断效率','日均大气透射率','Location','south');
% saveas(gcf, 'q1日均效率折线图.svg');
% 
% %%
% % 绘制春夏秋冬太阳方位、高度角图片
% close all;
% 
% % 绘制方位角
% alpha_s = [];
% gamma_s = [];
% leg = {};
% times = 8:0.1:16;
% month = [2, 4, 7, 11];
% for k = 1:4
%     i = month(k);
%     t = 0;
%     leg{k} = num2str(i) + "月21日";
%     for ST = times
%         D = days(q1_days(i) - SpringEquinox);
%         param = calDNI(D, ST , param);
%         t = t + 1;
%         alpha_s(k, t) = param.alpha_s;
%         gamma_s(k, t) = param.gamma_s;
%     end
% end
% plot(times, alpha_s(1, :), 'LineWidth', 1.5, 'Color', '#D4E157'); 
% hold on;
% plot(times, alpha_s(2, :), 'LineWidth', 1.5, 'Color','#42A5F5');
% plot(times, alpha_s(3, :), 'LineWidth', 2, 'Color','#F44336');
% plot(times, alpha_s(4, :), 'LineWidth', 1.5, 'Color','#26C6DA');
% legend(leg);
% xlabel('时间(h)','Interpreter','latex');
% ylabel('太阳高度角(rad)','Interpreter','latex');
% saveas(gcf, 'q1太阳高度角.svg');
% 
% % 绘制
% figure;
% plot(times, gamma_s(1, :), 'LineWidth', 1.5, 'Color', '#D4E157'); 
% hold on;
% plot(times, gamma_s(2, :), 'LineWidth', 1.5, 'Color','#42A5F5');
% plot(times, gamma_s(3, :), 'LineWidth', 2, 'Color','#F44336');
% plot(times, gamma_s(4, :), 'LineWidth', 1.5, 'Color','#26C6DA');
% legend(leg,'location','southeast');
% xlabel('时间(h)','Interpreter','latex');
% ylabel('太阳方位角(rad)','Interpreter','latex');
% saveas(gcf, 'q1方位角.svg');
% 
% %% 每一块镜子的年平均，并绘制热力图
% close all;
% load 问题一结果.mat
% 
% full_info = full_info';
% A = cat2one(full_info{1, 1});
% A(:, 3:end) = 0;
% for i = 1:size(full_info,1)
%     for j = 1:size(full_info, 2)
%         B = cat2one(full_info{i, j});
%         A(:, 3) = A(:, 3) + B(:, 32);
%         A(:, 4) = A(:, 4) + B(:, 32);
%     end
% end
% A(:, 3) = A(:, 3) ./ (size(full_info,1) * size(full_info,2));
% 
% % 绘制热力图
% % 绘制每一片镜子的总效率
% figure;
% scatter(A(:,1), A(:,2), 20, A(:,3), 'filled');
% colorbar;
% axis equal;
% saveas(gcf, "q1-年均效率.svg");
% %% 
% 
% 
% %% 函数
% 
% function A = cat2one(sub_mirrors)
%     A = [];
%     subs = length(sub_mirrors);
%     for i = 1:subs
%         A = [A; sub_mirrors{i}];
%     end
% end
