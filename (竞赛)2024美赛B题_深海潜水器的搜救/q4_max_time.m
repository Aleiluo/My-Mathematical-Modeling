function fitness = q4_max_time(routes, bps, full_info)
    % 输入:
    % route: 总路线
    % bp: 分割点
    % full_info: 所有求解信息

    % 返回fitness

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
%         points{i} = [];
%         points{i} = [points{i}; [0, cur_x, cur_y, 0]];
%         points{i} = [points{i}; [cur_t, cur_x, cur_y, cur_z]];

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
%             points{i} = [points{i}; [cur_t, cur_x, cur_y, cur_z]];
%             % 如果救援时间超过了沉底时间，返回一个大值
%             if cur_t > full_info{cur_route(j)}.sink_time
%                 fitness = 1e6;
%                 return
%             end
        end
        % 最后带着它的老婆孩子浮到水面
%         points{i} = [points{i}; [cur_t + points{i}(end, 4) / v_dive, ...
%             points{i}(end, 2), points{i}(end, 3), 0]];

        % 救完这一条路线后
        fitness = max(fitness, cur_t);
    end
end
