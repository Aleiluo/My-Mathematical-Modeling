function res = q2DWAsolve(droneA, droneB, dwaParam, map)
% 输入：两个无人机的所有参数，DWA参数，地图参数
% 输出：A、B无人机到达终点的耗时

Err_Return = 1e5;
Aarrive_time = Err_Return;
Barrive_time = Err_Return;
Aarrived = false;
Barrived = false;
A_w = [];
B_w = [];
A_score = [];
B_score = [];
A_dis2sta = [];
B_dis2sta = [];
AB_dis2o = [];

% 小优化：让无人机一开始朝向第一个目标点
droneA.curpos(3) = Angle(droneA.curpos(1:2), droneA.goal(1, :));
droneB.curpos(3) = Angle(droneB.curpos(1:2), droneB.goal(1, :));

% bug修复：靠近终点，t_s要变小
ToMidGoalErr = max(droneA.sampleDuration, droneB.sampleDuration) * max(droneA.v, droneB.v);
ToFinGoalErr = 2 * dwaParam.dt * max(droneA.v, droneB.v);

%绘图部分
% -----------------------------

if dwaParam.displayIter < 1e5
    update_counter = 0;
    close all
    fig = figure;
    % 图窗基本参数
    axis equal;
    axis([map.A(1) - 100, map.B(1) + 100, -1000, 1000]);
    hold on;
    grid on;
    box on;
    set(gcf, 'Color', [1,1,1]);
    set(gcf, 'Position', [100, 100, 800, 400]);
    set(gca, 'XMinorGrid', 'off'); % 关闭小刻度的网格
    set(gca, 'YMinorGrid', 'off'); % 关闭小刻度的网格
    ax = gca;
    ax.GridLineStyle = '--';

    % 添加基本元素

    % 设置圆的半径
    radius = 500;
    % 设置正方形的边长
    side_length = 2 * radius;
    % 绘制正方形并设置圆角属性
    rectangle('Position', [-radius, -radius, side_length, side_length], 'Curvature', [1, 1], 'EdgeColor', '#66BB6A', 'LineWidth', 1.7);
    % 描点
    scatter(droneA.curpos(1), droneA.curpos(2), 'o', 'filled', 'MarkerFaceColor', '#D81B60');
    text(droneA.curpos(1) - 30, droneA.curpos(2) - 100, '$A$', 'Interpreter', 'latex', 'FontSize', 14);
    scatter(droneB.curpos(1), droneB.curpos(2), 'o', 'filled', 'MarkerFaceColor', '#1E88E5');
    text(droneB.curpos(1) - 30, droneB.curpos(2) - 100, '$B$', 'Interpreter', 'latex', 'FontSize', 14);
    scatter(0, 0, 'o', 'filled', 'MarkerFaceColor', 'black', 'SizeData', 10);
    text(0 - 30, 0 - 100, '$O$', 'Interpreter', 'latex', 'FontSize', 14);
    % 添加标题和标签
    xlabel('X轴');
    ylabel('Y轴');
    
    line1_handle = [];
    line2_handle = [];
    line3_handle = [];
    line4_handle = [];

    % 记录上一个点
    lastApos = droneA.curpos;
    lastBpos = droneB.curpos;
    
    % gif动图部分
    image_counts = 0;
    im = {};
end

% -----------------------------

% 交替进行采样，交替一轮的时间是dt
for i = 1:dwaParam.maxIter
    % 预求出A、B的采样点(避免在后续重复计算)
    droneA.full_w = create_full_w(droneA.w_max, droneA.curpos(5), droneA.alpha_max, dwaParam.dt, dwaParam.dw);
    if Aarrived == true
        % 若是到达终点，取样点就是终点
        droneA.samplePoints = droneA.goal(end, :);
    else
        % 若没到达终点则用函数生成
        droneA.samplePoints = calSamplePos(droneA.curpos, droneA.full_w, droneA.sampleDuration, dwaParam.dt);
    end

    droneB.full_w = create_full_w(droneB.w_max, droneB.curpos(5), droneB.alpha_max, dwaParam.dt, dwaParam.dw);
    if Barrived == true
        droneB.samplePoints = droneB.goal(end, :);
    else
        droneB.samplePoints = calSamplePos(droneB.curpos, droneB.full_w, droneB.sampleDuration, dwaParam.dt);
    end
    % 绘图验证采样点是否计算正确
    % plot(droneB.samplePoints(:,1), droneB.samplePoints(:,2), '.',Color='green',markersize=1);
    
    % 求各无人机最佳角速度，并向前步进
    if Aarrived == false
        % 无人机A采样与更新
        [best_wA, score_A] = eval_bestw(droneA, droneB, dwaParam, map);
        if best_wA == inf
            error('无人机A无最佳角速度');
        end
        droneA.curpos = calNextPos(droneA.curpos ,best_wA, dwaParam.dt);
    end
    if Barrived == false
        % 无人机B采样与更新
        [best_wB, score_B] = eval_bestw(droneB, droneA, dwaParam, map);
        if best_wB == inf
            error('无人机B无最佳角速度');
        end
        droneB.curpos = calNextPos(droneB.curpos ,best_wB, dwaParam.dt);
    end

    % 因为评价中使用了近似的方法估计动态障碍物，所以两个无人机前进一步后要判断是否碰面
    if checkMeet(droneA.curpos(1:2), droneB.curpos(1:2), map.R)
        error('无人机A、B碰面');
    end

    A_w(i) = best_wA;
    B_w(i) = best_wB;
    A_score(i, :) = score_A;
    B_score(i, :) = score_B;
    A_dis2sta(i) = Dist(droneA.curpos(1:2), [0, 0]) - map.R;
    B_dis2sta(i) = Dist(droneB.curpos(1:2), [0, 0]) - map.R;
    k = (droneB.curpos(2) - droneA.curpos(2)) ./ (droneB.curpos(1) - droneA.curpos(1));
    AB_dis2o(i) = abs(-k .* droneA.curpos(1) + droneA.curpos(2)) ./ sqrt(1 + k.^2);

    % -----------------------------

    if dwaParam.displayIter < 1e5
        % 绘图
        line([lastApos(1), droneA.curpos(1)], [lastApos(2), droneA.curpos(2)], Color='#EC407A', LineWidth=1.5);
        line([lastBpos(1), droneB.curpos(1)], [lastBpos(2), droneB.curpos(2)], Color='#42A5F5', LineWidth=1.5);
        lastApos = droneA.curpos;
        lastBpos = droneB.curpos;
        % title(num2str(i));
        if dwaParam.drawTangent
            % 画切线
            [kA1, kA2] = calCutSlope(droneA.curpos(1:2), map.R);
            [kB1, kB2] = calCutSlope(droneB.curpos(1:2), map.R);
            if ~isempty(line1_handle)
                delete(line1_handle);
            end
            if ~isempty(line2_handle)
                delete(line2_handle);
            end
            if ~isempty(line3_handle)
                delete(line3_handle);
            end
            if ~isempty(line4_handle)
                delete(line4_handle);
            end
            linelen = 5000;
            % 画A的切线
            line1_handle = plot([droneA.curpos(1) - linelen, droneA.curpos(1) + linelen], ...
                [-linelen * kA1 + droneA.curpos(2), linelen * kA1 + droneA.curpos(2)], ...
                'Color', '#FF7043', 'LineWidth', 0.9, 'LineStyle', '--');
            line2_handle = plot([droneA.curpos(1) - linelen, droneA.curpos(1) + linelen], ...
                [-linelen * kA2 + droneA.curpos(2), linelen * kA2 + droneA.curpos(2)], ...
                'Color', '#FF7043', 'LineWidth', 0.9, 'LineStyle', '--');
            % 画B的切线
            line3_handle = plot([droneB.curpos(1) + linelen, droneB.curpos(1) - linelen], ...
                [linelen * kB1 + droneB.curpos(2), -linelen * kB1 + droneB.curpos(2)], ...
                'Color', '#AB47BC', 'LineWidth', 0.9, 'LineStyle', '--');
            line4_handle = plot([droneB.curpos(1) + linelen, droneB.curpos(1) - linelen], ...
                [linelen * kB2 + droneB.curpos(2), -linelen * kB2 + droneB.curpos(2)], ...
                'Color', '#AB47BC', 'LineWidth', 0.9, 'LineStyle', '--');
        end
        
        %title(['It:', num2str(i), ' | A2goal:', num2str(dis2goalA), ' | B2goal:', num2str(dis2goalB)]);

        % 判断是否需要更新图形
        update_counter = update_counter + 1;
        if update_counter >= dwaParam.displayIter
            drawnow;
            update_counter = 0;
            % 动图
            if dwaParam.gif == true
                image_counts = image_counts + 1;
                frame = getframe(fig);
                im{image_counts} = frame2im(frame);
            end
        end
    end

    % -----------------------------

    % 算距离
    dis2goalA = Dist(droneA.curpos(1:2), droneA.goal(droneA.curgoal,:));
    dis2goalB = Dist(droneB.curpos(1:2), droneB.goal(droneB.curgoal,:));

    if dis2goalA < ToMidGoalErr && droneA.curgoal == size(droneA.goal, 1)
        % 此刻要更改向前采样时间逐渐逼近终点
        droneA.sampleDuration = min(droneA.sampleDuration, (dis2goalA - ToFinGoalErr) / droneA.v);
        if dis2goalA < ToFinGoalErr && Aarrived == false
            Aarrived = true;
            Aarrive_time = i * dwaParam.dt + dis2goalA / droneA.v;
            AfinX = droneA.goal(droneA.curgoal, 1);
            AfinY = droneA.goal(droneA.curgoal, 2);
            curpos = [AfinX, AfinY];
            droneA.curpos(1:2) = curpos;
            if dwaParam.displayIter < 1e5
                line([lastApos(1), AfinX], ...
                     [lastApos(2), AfinY], Color='#EC407A', LineWidth=1.5);
                if dwaParam.drawTangent
                    % 画切线
                    [kA1, kA2] = calCutSlope(curpos(1:2), map.R);
                    if ~isempty(line1_handle)
                        delete(line1_handle);
                    end
                    if ~isempty(line2_handle)
                        delete(line2_handle);
                    end
                    linelen = 5000;
                    % 画A的切线
                    line1_handle = plot([curpos(1) - linelen, curpos(1) + linelen], ...
                        [-linelen * kA1 + curpos(2), linelen * kA1 + curpos(2)], ...
                        'Color', '#FF7043', 'LineWidth', 0.9, 'LineStyle', '--');
                    line2_handle = plot([curpos(1) - linelen, curpos(1) + linelen], ...
                        [-linelen * kA2 + curpos(2), linelen * kA2 + curpos(2)], ...
                        'Color', '#FF7043', 'LineWidth', 0.9, 'LineStyle', '--');
                end
            end
        end
    end

    if dis2goalB < ToMidGoalErr && droneB.curgoal == size(droneB.goal, 1)
        % 此刻要更改向前采样时间逐渐逼近终点
        droneB.sampleDuration = min(droneB.sampleDuration, (dis2goalB - ToFinGoalErr) / droneB.v);
        if dis2goalB < ToFinGoalErr && Barrived == false
            Barrived = true;
            Barrive_time = i * dwaParam.dt + dis2goalB / droneA.v;
            BfinX = droneB.goal(droneB.curgoal, 1);
            BfinY = droneB.goal(droneB.curgoal, 2);
            curpos = [BfinX, BfinY];
            droneB.curpos(1:2) = curpos;
            if dwaParam.displayIter < 1e5
                line([lastBpos(1), BfinX], [lastBpos(2), BfinY], Color='#42A5F5', LineWidth=1.5);
                if dwaParam.drawTangent
                    [kB1, kB2] = calCutSlope(curpos, map.R);
                    if ~isempty(line3_handle)
                        delete(line3_handle);
                    end
                    if ~isempty(line4_handle)
                        delete(line4_handle);
                    end
                    linelen = 5000;
                    % 画B的切线
                    line3_handle = plot([curpos(1) + linelen, curpos(1) - linelen], ...
                        [linelen * kB1 + curpos(2), -linelen * kB1 + curpos(2)], ...
                        'Color', '#AB47BC', 'LineWidth', 0.9, 'LineStyle', '--');
                    line4_handle = plot([curpos(1) + linelen, curpos(1) - linelen], ...
                        [linelen * kB2 + curpos(2), -linelen * kB2 + curpos(2)], ...
                        'Color', '#AB47BC', 'LineWidth', 0.9, 'LineStyle', '--');
                end
            end
        end
    end

    % 是否抵达中间目标点
    if dis2goalA < ToMidGoalErr && droneA.curgoal < size(droneA.goal, 1)
        droneA.curgoal = droneA.curgoal + 1;
    end

    if dis2goalB < ToMidGoalErr && droneB.curgoal < size(droneB.goal, 1)
        droneB.curgoal = droneB.curgoal + 1;
    end

    % A和B都到达了各自的终点
    if Aarrived && Barrived
        break;
    end


end


if dwaParam.displayIter < 1e5
    res.fig = fig;
    % gif
    if dwaParam.gif == true
        for i = 1:image_counts
            [Image, Image_map] = rgb2ind(im{i},256);
            if i == 1
                imwrite(Image,Image_map,dwaParam.gif_name,"gif","LoopCount",Inf,"DelayTime",dwaParam.gif_dt);
            else
                imwrite(Image,Image_map,dwaParam.gif_name,"gif","WriteMode","append","DelayTime",dwaParam.gif_dt);
            end
        end
    end
end

% 整理返回信息
res.A_time = Aarrive_time;
res.B_time = Barrive_time;
res.A_w = A_w;
res.B_w = B_w;
res.A_score = A_score;
res.B_score = B_score;
res.A_dis2sta = A_dis2sta;
res.B_dis2sta = B_dis2sta;
res.AB_dis2o = AB_dis2o;
end

function [best_w, score] = eval_bestw(main_drone, other_drone, dwaParam, map)
    % 判断dt后无人机的位置是否会撞到静态障碍物上
    % 该位置与角速度无关
    nxtPoint = [main_drone.curpos(1) + main_drone.curpos(4) * cos(main_drone.curpos(3)) * dwaParam.dt, ...
                main_drone.curpos(2) + main_drone.curpos(4) * sin(main_drone.curpos(3)) * dwaParam.dt];

    if (sqrt(nxtPoint(1)^2 + nxtPoint(2)^2) - map.R) < 0
        best_w = inf;
        return;
    end
    
    % 判断下一步位置是否会撞在动态障碍物上
    % 第一步：选取其他无人机最佳采样点
    SP2cycle = sqrt(other_drone.samplePoints(:, 1).^2 + other_drone.samplePoints(:, 2).^2) - map.R;
    % 选择在圆外且离圆最近的采样点(做最激进的估计)
    positive_idx = SP2cycle > 0;
    [~, min_positive_idx] = min(SP2cycle(positive_idx));
    tmp_points = other_drone.samplePoints(positive_idx, 1:2);
    bestSamplePoint = tmp_points(min_positive_idx, :);
    % 第二步：判断nxtPoint与该采样点会不会碰面
    if checkMeet(bestSamplePoint, nxtPoint, map.R)
        best_w = inf;
        return;
    end

    % 确保不会撞后，正式开始评估

    M = 1e5;
    w_counts = length(main_drone.full_w);
    cur_goal = main_drone.goal(main_drone.curgoal, :);
    drone_score = -Inf(w_counts, 5);
    drone_score(:, 5) = main_drone.full_w;
    
    % 朝向评价
    theta = atan2(cur_goal(2) - main_drone.samplePoints(:, 2), ...
                  cur_goal(1) - main_drone.samplePoints(:, 1));
    drone_score(:, 1) = pi - abs(atan2(sin(main_drone.samplePoints(:, 3) - theta), ...
                                       cos(main_drone.samplePoints(:, 3) - theta)));

    % 到终点距离评价
    drone_score(:, 2) = M - Dist(main_drone.samplePoints(:, 1:2), cur_goal);

    % 静态障碍物评价
    drone_score(:, 3) = min(Dist(main_drone.samplePoints(:, 1:2), [0, 0]) - map.R, ...
                            main_drone.minDis2StaticObstacle);
    % 将在圆内的采样点的得分设为负无穷
    drone_score(drone_score(:, 3) < 0, :) = -Inf;
    
    % 动态障碍物评价
    drone_score(:, 4) = min(calDis2cut(bestSamplePoint, main_drone.samplePoints(:, 1:2), map.R), ...
                            main_drone.minDis2DynamicObstacle);
    % 将见面的采样点的得分设为负无穷
    isMeet = checkMeet(bestSamplePoint, main_drone.samplePoints(:, 1:2), map.R);
    drone_score(isMeet, :) = -Inf;

    % 去掉含-Inf的行
    drone_score = drone_score(drone_score(:, 1) ~= -Inf, :);

    % 如果去掉无用行后无路可走，说明本次优化参数作废，返回一个大值
    if size(drone_score, 1) == 0
        best_w = inf;
        return;
    end

    % 标准化处理
    drone_norm_score = zscore(drone_score(:, 1:4));

    % 计算得分
    drone_weight_score = drone_norm_score * [main_drone.headingWeight; ...
                                             main_drone.dis2goalWeight; ...
                                             main_drone.StaticDisWeight; ...
                                             main_drone.DynamicDisWeight];

    % 获取最大得分并更新
    [~, index] = max(drone_weight_score);
    best_w = drone_score(index, 5);
    score = drone_norm_score(index, 1:4);

%     if main_drone.name == 'B'
%         fprintf('%.2f,%.2f,%.2f,%.2f\n',drone_score(index, 1),drone_score(index, 2),drone_score(index, 3),drone_score(index, 4));
%     end
    
end

function samplePoints = calSamplePos(cur_point, full_w, sampleDuration, dt)
    % 计算轨迹：该函数在DWA算法中将被多次调用，注意开销
    % 输入：当前无人机位置，所有的角速度，向后采样时间，时间分辨率
    % 返回：一行一个采样点，采样点个数与输入的角速度个数一致
    n = floor(sampleDuration / dt);
    samplePoints = zeros(length(full_w), 5);
    samplePoints(:, 5) = full_w;
    w_dt = dt * full_w;
    % 去掉含0的行
    nonzero_idx = w_dt ~= 0;
    w_dt = w_dt(nonzero_idx);
    deno = 2 * sin(0.5 * w_dt);
    
    % 区分w=0和w~=0时的公式
    samplePoints(nonzero_idx, 1) = cur_point(1) + dt .* cur_point(4) .* (cos(cur_point(3)) + ( ...
                                   sin((n - 0.5) .* w_dt + cur_point(3)) - sin(0.5 .* w_dt + cur_point(3)) ) ./ deno);
    samplePoints(~nonzero_idx , 1) = cur_point(1) + n * dt * cur_point(4) * cos(cur_point(3));

    samplePoints(nonzero_idx, 2) = cur_point(2) + dt .* cur_point(4) .* (sin(cur_point(3)) + ( ...
                                   cos(0.5 .* w_dt + cur_point(3)) - cos((n - 0.5) .* w_dt + cur_point(3)) ) ./ deno);
    samplePoints(~nonzero_idx, 2) = cur_point(2) + n * dt * cur_point(4) * sin(cur_point(3));

    samplePoints(:, 3) = cur_point(3) + n * dt * full_w;
    samplePoints(:, 4) = cur_point(4);
end

function nextpos = calNextPos(pos, w, dt)
    % pos: 当前位置[x, y, theta, v, w]
    % w: 1个角速度
    % 返回dt后的所有位置
    nextpos = [pos(1) + pos(4) * cos(pos(3)) * dt, ...
               pos(2) + pos(4) * sin(pos(3)) * dt, ...
               pos(3) + w * dt, ...
               pos(4), ...
               w];
end

function ismeet = checkMeet(node1, node2, R)
    % 输入node1：一个或多个点，node2：一个或多个点，R：圆半径
    % 返回：布尔列向量
    k = (node2(:, 2) - node1(:, 2)) ./ (node2(:, 1) - node1(:, 1));
    ismeet = abs(-k .* node1(:, 1) + node1(:, 2)) ./ sqrt(1 + k.^2) > R;
end

function dis2cut = calDis2cut(node1, node2, R)
    % 输入：
    % node1:一行，表示一个点。
    % node2:多行，表示多个点。
    % 返回：
    % 一个列向量，表示每一个node2到node1与圆构成两条切线的最小距离。
    corfA = node1(1)^2 - R^2;
    corfB = -2 * node1(1) * node1(2);
    corfC = node1(2)^2 - R^2;
    k1 = (-corfB + sqrt(corfB^2 - 4 * corfA * corfC)) / (2 * corfA);
    k2 = (-corfB - sqrt(corfB^2 - 4 * corfA * corfC)) / (2 * corfA);
    dis1 = abs(k1 .* node2(:, 1) - node2(:, 2) - k1 .* node1(1) + node1(2)) ./ sqrt(1 + k1.^2);
    dis2 = abs(k2 .* node2(:, 1) - node2(:, 2) - k2 .* node1(1) + node1(2)) ./ sqrt(1 + k2.^2);
    dis2cut = min(dis1, dis2);
end

function [k1, k2] = calCutSlope(node1, R)
    % 计算node1与圆相切的两条线的斜率k1, k2
    corfA = node1(1)^2 - R^2;
    corfB = -2 * node1(1) * node1(2);
    corfC = node1(2)^2 - R^2;
    k1 = (-corfB + sqrt(corfB^2 - 4 * corfA * corfC)) / (2 * corfA);
    k2 = (-corfB - sqrt(corfB^2 - 4 * corfA * corfC)) / (2 * corfA);
end

function the = Angle(node1, node2)
    the = atan2(node2(2) - node1(2), node2(1) - node1(1));
end

function dist = Dist(node1, node2)
    % 计算欧几里德距离
    dist = sqrt((node1(:, 1) - node2(:, 1)).^2 + (node1(:, 2) - node2(:, 2)).^2);
end

function full_w = create_full_w(w_max, cur_w, alpha_max, dt, dw)
    % 输入：无人机最大角速度，当前角速度，最大角加速度，时间分辨率，角速度分辨率
    % 第一步：确认角速度范围
    w_scale = [max(-w_max, cur_w - alpha_max * dt), min(w_max, cur_w + alpha_max * dt)];
    % 第二步：根据范围和角速度分辨率生成离散角速度
    full_w = dw * [ceil(w_scale(1) / dw):1:floor(w_scale(2) / dw)]';
end

function mat = norm_mat(mat)
    % 检查每一列是否全为0
    col_sums = sum(mat);
    zero_cols = col_sums == 0;
    % 避免除以0，将全为0的列保持不变，其他列进行除法
    mat(:, ~zero_cols) = mat(:, ~zero_cols) ./ col_sums(~zero_cols);
end

