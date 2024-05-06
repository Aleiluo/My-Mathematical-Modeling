function res = q1DWAsolve(droneA, droneB, dwaParam, map, cutLine)
% 参数

% 本程序仅解决第一问，是一个极其不完备的算法，所有的改进在第二问及以后

update_counter = 0;
steps = 0;
curtime = 0;
resultpos = zeros(dwaParam.maxIter, 2);
resultbest_wA = zeros(dwaParam.maxIter, 1);
resultMinDis = zeros(dwaParam.maxIter, 1);
A_dis2sta = [];
AB_dis2o = [];

fig = figure;
% 设置圆的半径
radius = 500;
% 设置正方形的边长
side_length = 2 * radius;
% 绘制正方形并设置圆角属性
rectangle('Position', [-radius, -radius, side_length, side_length], 'Curvature', [1, 1], 'EdgeColor', '#66BB6A', 'LineWidth', 1.7);
hold on;
% 描点
scatter(droneA.curpos(1), droneA.curpos(2), 'o', 'filled', 'MarkerFaceColor', '#D81B60');
text(droneA.curpos(1) - 30, droneA.curpos(2) - 100, '$A$', 'Interpreter', 'latex', 'FontSize', 14);
scatter(droneB.curpos(1), droneB.curpos(2), 'o', 'filled', 'MarkerFaceColor', '#1E88E5');
text(droneB.curpos(1) - 30, droneB.curpos(2) - 100, '$B$', 'Interpreter', 'latex', 'FontSize', 14);
scatter(0, 0, 'o', 'filled', 'MarkerFaceColor', 'black', 'SizeData', 10);
text(0 - 30, 0 - 100, '$O$', 'Interpreter', 'latex', 'FontSize', 14);
% 设置坐标轴范围
axis equal;
box on;
% 设置坐标轴为虚线网格
grid on;
set(gcf, 'Color', [1,1,1]);
set(gcf, 'Position', [100, 100, 800, 400]);
set(gca, 'XMinorGrid', 'off'); % 关闭小刻度的网格
set(gca, 'YMinorGrid', 'off'); % 关闭小刻度的网格

image_counts = 0;
im = {};

% 设置网格线为虚线
ax = gca;
ax.GridLineStyle = '--';
axis([-1200, 3700, -1000, 1000]);


line1_handle = [];
line2_handle = [];
line3_handle = [];
line4_handle = [];

% bug修复：靠近终点，t_s要变小
ToMidGoalErr = dwaParam.sampleDuration * droneA.v;
ToFinGoalErr = 2 * dwaParam.dt * droneA.v;

for i = 1:dwaParam.maxIter
    % 采样得到无人机A的角速度
    [best_wA, bestStdScore] = eval_bestw(droneA, droneB, dwaParam, map, curtime);
    % 更新位置、记录结果
    droneA.curpos = calNextPos(droneA.curpos ,best_wA, dwaParam.dt);
    curtime = curtime + dwaParam.dt;
    droneB.curpos(1:2) = getdroneBpos(curtime, droneB.v, map);
    resultpos(i, :) = droneA.curpos(1:2);
    resultbest_wA(i) = best_wA;
    A_dis2sta(i) = sqrt(droneA.curpos(1)^2+droneA.curpos(2)^2) - map.R;
    k = (droneB.curpos(2) - droneA.curpos(2)) ./ (droneB.curpos(1) - droneA.curpos(1));
    AB_dis2o(i) = abs(-k .* droneA.curpos(1) + droneA.curpos(2)) ./ sqrt(1 + k.^2);
    stdScore(i, :) = bestStdScore;

    % 绘图
    plot(droneA.curpos(1), droneA.curpos(2), '.',Color='#EC407A',markersize=3);
    plot(droneB.curpos(1), droneB.curpos(2), '.',Color='#42A5F5',markersize=3);
    if cutLine == true
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
    
    

    % 算距离
    dis2goal = sqrt((droneA.curpos(1)-droneA.goal(1))^2 + (droneA.curpos(2)-droneA.goal(2))^2);
    %title(['第 ', num2str(i), ' 次迭代，距离终点', num2str(dis2goal(1)), '米']);

    % 判断是否需要更新图形
    update_counter = update_counter + 1;
    if update_counter >= dwaParam.displayIter
        drawnow;
        update_counter = 0;
        if dwaParam.gif == true
            frame = getframe(fig);
            image_counts = image_counts + 1;
            im{image_counts} = frame2im(frame);
        end
    end

    % 寻优结束判定
    if dis2goal < ToMidGoalErr
        dwaParam.sampleDuration = min(dwaParam.sampleDuration, (dis2goal - ToFinGoalErr) / droneA.v);
        if dis2goal < ToFinGoalErr
            steps = i;
            break;
        end
    end
    

    
end

if dwaParam.gif == true
    for i = 1:image_counts
        [Image, Image_map] = rgb2ind(im{i},256);
        if i == 1
            imwrite(Image,Image_map,dwaParam.gif_name,"gif","LoopCount",Inf,"DelayTime",dwaParam.git_dt);
        else
            imwrite(Image,Image_map,dwaParam.gif_name,"gif","WriteMode","append","DelayTime",dwaParam.git_dt);
        end
    end

end

% 出图出结果
res.Aarrive_time = steps * dwaParam.dt + dis2goal / droneA.v;
res.Barrive_time = map.t3;
res.A_w = resultbest_wA(1:steps,:);
res.A_dis = resultMinDis(1:steps,:);
res.A_score = stdScore(1:steps,:);
res.A_dis2sta = A_dis2sta;
res.AB_dis2o = AB_dis2o;
end

function [best_wA, bestStdScore] = eval_bestw(droneA, droneB, dwaParam, map, curtime)
    M = 1e5;

    droneAdegScale = [max(-droneA.w_max, droneA.curpos(5) - droneA.alpha_max * dwaParam.dt), ...
                      min(droneA.w_max, droneA.curpos(5) + droneA.alpha_max * dwaParam.dt)];
%     droneBdegScale = [droneB.curpos(3) - droneB.alpha_max * dwaParam.dt, ...
%                       droneB.curpos(3) + droneB.alpha_max * dwaParam.dt];
    droneAeval = zeros(floor((droneAdegScale(2) - droneAdegScale(1)) / dwaParam.dw), 5);
    insertRow = 0;

    % 获取当前时刻无人机B的坐标
    Bpos = getdroneBpos(curtime + dwaParam.sampleDuration, droneB.v, map);
    % 计算两个切线的斜率
    corfA = Bpos(1)^2 - 500^2;
    corfB = -2 * Bpos(1) * Bpos(2);
    corfC = Bpos(2)^2 - 500^2;
    k1 = (-corfB + sqrt(corfB^2 - 4 * corfA * corfC)) / (2 * corfA);
    k2 = (-corfB - sqrt(corfB^2 - 4 * corfA * corfC)) / (2 * corfA);

    for wA = droneAdegScale(1):dwaParam.dw:droneAdegScale(2)
        % 对于每一个角度，计算sampleDuration后的位置
        traj = calSamplePos(droneA.curpos, wA, dwaParam);

        % 对采样点进行评估
        samplepos = traj(end, :);

        % 朝向评估
        theta = angle([samplepos(1), samplepos(2)], droneA.goal(1:2));
        heading = pi - abs(atan2(sin(samplepos(3) - theta), ...
                                 cos(samplepos(3) - theta)));
        
        % 到终点距离评估
        dis2goal = M-sqrt((samplepos(1)-droneA.goal(1))^2 + (samplepos(2)-droneA.goal(2))^2);

        % 障碍评估
        dis2cut1 = abs(k1 * samplepos(1) - samplepos(2) - k1 * Bpos(1) + Bpos(2)) / sqrt(1 + k1^2);
        dis2cut2 = abs(k2 * samplepos(1) - samplepos(2) - k2 * Bpos(1) + Bpos(2)) / sqrt(1 + k2^2);
        % 判断采样点与切线的方位
%         if k1 * samplepos(1) - samplepos(2) - k1 * Bpos(1) + Bpos(2) > 0
%             continue
%         end
        % 到圆的距离
        dis2circle = sqrt(samplepos(1)^2 + samplepos(2)^2) - map.R;
        if dis2circle < 0
            continue
        end
        % 到各边界的距离
%         dis2upper = droneA.upper - samplepos(2);
%         dis2lower = samplepos(2) - droneA.lower;
%         dis2left = samplepos(1) - droneA.left;
%         dis2right = droneA.right - samplepos(1);
        % 取最小距离
%         min_dis = min([dis2cut1, dis2cut2, dis2circle, ...
%                        dis2upper, dis2lower, dis2left, dis2right]);
        min_dis = min([dis2cut1, dis2cut2, dis2circle]);
        % 记录数据
        adjmin_dis = min_dis;
        if adjmin_dis > dwaParam.minDis2Obstacle
            adjmin_dis = dwaParam.minDis2Obstacle;
        end
        

        % 因为是匀速，所以没有速度评估，也没有速度与制动评估

        insertRow = insertRow + 1;
        droneAeval(insertRow, :) = [heading, adjmin_dis, dis2goal, wA, min_dis];
    end

    droneAeval = droneAeval(1:insertRow,:);

%     norm_droneAeval = norm_mat(droneAeval(:, 1:3));

    norm_droneAeval = zscore(droneAeval(:, 1:3));

    % 计算得分
    droneAscore = norm_droneAeval * [dwaParam.headingWeight; ...
                                        dwaParam.distanceWeight; ...
                                        dwaParam.dis2goalWeight];

    % 获取最大得分并更新
    [~, index] = max(droneAscore);
    best_wA = droneAeval(index, 4);
    bestStdScore = norm_droneAeval(index, 1:3);

    % fprintf('%.2f,%.2f,%.2f,%.2f\n',droneAeval(index, 1),droneAeval(index, 2),droneAeval(index, 3),droneAeval(index, 4));

end

function traj = calSamplePos(pos, w, dwaParam)
    % 预分配轨迹空间
    steps = floor(dwaParam.sampleDuration / dwaParam.dt);
    traj = zeros(steps,5);
    traj(1,:) = pos;
    % 循环求轨迹
    for step = 2:steps
        traj(step, :) = calNextPos(traj(step - 1, :), w, dwaParam.dt);
    end
end

function nextpos = calNextPos(pos, w, dt)
    % pos:当前位置[x, y, theta, v, w]
    nextpos =  [pos(1) + pos(4) * cos(pos(3)) * dt, ...
                pos(2) + pos(4) * sin(pos(3)) * dt, ...
                pos(3) + w * dt, ...
                pos(4), ...
                w];
end

function theta = angle(node1, node2)
    theta = atan2(node2(2) - node1(2), node2(1) - node1(1));
end

function Bpos = getdroneBpos(t, speed, map)
    % 确认B运动路径关于时间t的参数方程
    if t < map.t1
        x = map.B(1) - t * cos(map.thetaB) * speed;
        y = map.B(2) + t * sin(map.thetaB) * speed;
    elseif map.t1 <= t && t < map.t2
        % 角速度
        w = speed / map.R;
        theta0 = abs(atan(map.cutB(2) / map.cutB(1)));
        theta = theta0 + w * (t - map.t1);
        x = map.R * cos(theta);
        y = map.R * sin(theta);
    elseif map.t2 <= t && t < map.t3
        x = map.cutA(1) - (t - map.t2) * cos(map.thetaA) * speed;
        y = map.cutA(2) - (t - map.t2) * sin(map.thetaA) * speed;
    elseif map.t3 <= t
        % t3时刻后无人机都在A点
        x = map.A(1);
        y = map.A(2);
    end
    Bpos = [x, y];
end

function [k1, k2] = calCutSlope(node1, R)
    % 计算node1与圆相切的两条线的斜率k1, k2
    corfA = node1(1)^2 - R^2;
    corfB = -2 * node1(1) * node1(2);
    corfC = node1(2)^2 - R^2;
    k1 = (-corfB + sqrt(corfB^2 - 4 * corfA * corfC)) / (2 * corfA);
    k2 = (-corfB - sqrt(corfB^2 - 4 * corfA * corfC)) / (2 * corfA);
end

function mat = norm_mat(mat)
    % 检查每一列是否全为0
    col_sums = sum(mat);
    zero_cols = col_sums == 0;
    % 避免除以0，将全为0的列保持不变，其他列进行除法
    mat(:, ~zero_cols) = mat(:, ~zero_cols) ./ col_sums(~zero_cols);
end

