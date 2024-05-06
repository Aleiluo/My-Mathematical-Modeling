function sub_mirrors = cal_target(sub_mirrors, param)
subs = length(sub_mirrors);

for i = 1:subs
    if isempty(sub_mirrors{i})
        continue
    end
    sublen = size(sub_mirrors{i}, 1);
    for j = 1:sublen
        % 一共有四类阴影
        shadow_cnts = 0;
        shadow_area = 0;
        shadow_points = {};
        for k = 1:4
            shadow_points{k} = [];
        end
        % 入射、反射光线向量
        r_vec = [0 - sub_mirrors{i}(j, 1), 0 - sub_mirrors{i}(j, 2), param.Hz - sub_mirrors{i}(j, 3)];
        tot_vec = [param.light_vec; r_vec];

        %% --------------------
        % 一次性计算所有的遮挡点
        for k = j + 1:sublen
            % 计算两者距离
            if sqrt((sub_mirrors{i}(j, 1) - sub_mirrors{i}(k, 1))^2 + (sub_mirrors{i}(j, 2) - sub_mirrors{i}(k, 2))^2) > param.max_dis
                % 使用两者距离公式跳过
                continue
            end
            if abs(sub_mirrors{i}(j, 6) - sub_mirrors{i}(k, 6)) > param.max_dis
                % 使用径向距离打断循环
                break;
            end
            % 对入射、反射都有遮挡
            for r = 1:2
                [p14, p5] = AtransB(sub_mirrors{i}(j, :), sub_mirrors{i}(k, :), tot_vec(r, :));
                for t = 1:4
                    % 4个点只可能有1个点位于反射镜内
                    if judge_in_area(sub_mirrors{i}(j, :), p14(t, :))
                        % 通过4个点中点坐标位置判断遮蔽阴影类型
                        shadow_cnts = shadow_cnts + 1;
                        st = get_shadow_type(p14(t, :), p5);
                        shadow_points{st} = [shadow_points{st}; p14(t, :)];
                        break;
                    end
                end
            end
        end
        %% ---------------------

        % 计算遮蔽效率
        if shadow_cnts > 0
            sub_mirrors{i}(j, 27) = 1 - cal_shadow_area(shadow_points, sub_mirrors{i}(j, 4), sub_mirrors{i}(j, 5)) / ...
                (sub_mirrors{i}(j, 4) * sub_mirrors{i}(j, 5));
        else
            sub_mirrors{i}(j, 27) = 1;
        end

        % 顺便计算集热器截断效率
        sub_mirrors{i}(j, 28) = cal_eta_trunc(sub_mirrors{i}(j, :), param);
    end

    % 余弦效率
    sub_mirrors{i}(:, 29) = sub_mirrors{i}(:, 9:11) * param.light_vec';

    % 大气透射效率
    x = sub_mirrors{i}(:, 1);
    y = sub_mirrors{i}(:, 2);
    z = sub_mirrors{i}(:, 3);
    dHR = sqrt((0 - x).^2 + (0 - y).^2 + (param.Hz - z).^2);
    sub_mirrors{i}(:, 30) = 0.99321 - 0.0001176 .* dHR + 1.97 .* 1e-8 .* dHR.^2;
    
    % 镜面反射效率
    sub_mirrors{i}(:, 31) = 0.92;

    % 光学效率
    sub_mirrors{i}(:, 32) = prod(sub_mirrors{i}(:, 27:31), 2);

    % 计算面积与效率的乘积
    sub_mirrors{i}(:, 33) = sub_mirrors{i}(:, 4) .* sub_mirrors{i}(:, 5) .* sub_mirrors{i}(:, 32);
end
    
end

function flag = judge_in_area(mirror_info, p)
    % 判断p是否在矩形中
    % 宽度--x, 长度--y
    if p(1) >= -mirror_info(5)/2 && p(1) <= mirror_info(5)/2 && p(2) >= -mirror_info(4)/2 && p(2) <= mirror_info(4)/2
        flag = true;
    else
        flag = false;
    end
end

function k = get_shadow_type(p, p_mid)
    % 判断在矩形中的点p的阴影类型
    % 返回k = 1:右上, 2:左上, 3:左下, 4:右下
    if p_mid(1) > p(1) && p_mid(2) > p(2)
        k = 1;
    elseif p_mid(1) < p(1) && p_mid(2) > p(2)
        k = 2;
    elseif p_mid(1) < p(1) && p_mid(2) < p(2)
        k = 3;
    elseif p_mid(1) > p(1) && p_mid(2) < p(2)
        k = 4;
    else
        k = -1;
    end
end





