function sub_mirrors = cal_mirror_info(sub_mirrors, param)
% 根据太阳方位计算反射镜的角度

subs = length(sub_mirrors);

for i = 1:subs
    if isempty(sub_mirrors{i})
        continue
    end
    % 使用向量化计算
    x = sub_mirrors{i}(:, 1);
    y = sub_mirrors{i}(:, 2);
    z = sub_mirrors{i}(:, 3);

    % 反射光线计算
    sublen = length(x);
    r = [0 - x, 0 - y, param.Hz - z];
    r = r ./ sqrt(r(:, 1).^2 + r(:, 2).^2 + r(:, 3).^2);
    a_theta2 = sqrt((1 + param.light_vec * r') ./ 2)';
    n = (param.light_vec + r) ./ (2 .* a_theta2);  

    % 计算俯仰角(注意，真的俯仰角要用90度减去该值)
    tz = pi / 2 - asin(n(:,3));
    sub_mirrors{i}(:, 7) = tz;
    % 计算方位角
    ts = atan(n(:, 2) ./ n(:, 1));
    sub_mirrors{i}(:, 8) = ts;

%     % 计算各定日镜vec_x
%     sub_mirrors{i}(:, 9:11) = [sin(ts), -cos(ts), zeros(sublen, 1)];
%     % 计算各定日镜vec_y
%     sub_mirrors{i}(:, 12:14) = [-cos(ts) .* tan(tz), -sin(ts) .* tan(tz), ones(sublen, 1)];
%     % 计算各定日镜vec_z
%     sub_mirrors{i}(:, 15:17) = [cos(ts), sin(ts), tan(tz)];
    % 记录法向量
    sub_mirrors{i}(:, 9:11) = n;
    % 计算四点坐标
    for j = 1:sublen
        T = [-sin(tz(j)), -sin(ts(j)) .* cos(tz(j)), cos(ts(j)) .* cos(tz(j)); ...
              cos(tz(j)), -sin(ts(j)) .* sin(tz(j)), cos(ts(j)) .* sin(tz(j)); ...
              0,          cos(ts(j)),                -sin(ts(j))];
        m = sub_mirrors{i}(j, :);
        points = [0.5 * m(5), 0.5 * m(4), 0; ...
                  -0.5 * m(5), 0.5 * m(4), 0; ...
                  -0.5 * m(5), -0.5 * m(4), 0; ...
                  0.5 * m(5), -0.5 * m(4), 0; ...
                  0, 0, 0; ...
                 ]';
        sub_mirrors{i}(j, 12:26) = reshape(T * points + [x(j); y(j); z(j)], 1, 15);
    end
%     % 右上
%     sub_mirxrors{i}(:, 5) = x + 0.5 .* param.width .* cos(ts) - 0.5 * param.width .* sin(ts);
%     sub_mirxrors{i}(:, 6) = y + 0.5 .* param.len .* cos(ts) + 0.5 .* param.width .* sin(ts);
%     sub_mirxrors{i}(:, 7) = param.height + 0.5 .* param.len .* sin(tz);
% 
%     % 左上
%     sub_mirxrors{i}(:, 8) = x - 0.5 .* param.width .* cos(ts) - 0.5 * param.width .* sin(ts);
%     sub_mirxrors{i}(:, 9) = y + 0.5 .* param.len .* cos(ts) - 0.5 .* param.width .* sin(ts);
%     sub_mirxrors{i}(:, 10) = param.height + 0.5 .* param.len .* sin(tz);
% 
%     % 左下
%     sub_mirxrors{i}(:, 11) = x - 0.5 .* param.width .* cos(ts) + 0.5 * param.width .* sin(ts);
%     sub_mirxrors{i}(:, 12) = y - 0.5 .* param.len .* cos(ts) - 0.5 .* param.width .* sin(ts);
%     sub_mirxrors{i}(:, 13) = param.height - 0.5 .* param.len .* sin(tz);
% 
%     % 右下
%     sub_mirxrors{i}(:, 14) = x + 0.5 .* param.width .* cos(ts) + 0.5 * param.width .* sin(ts);
%     sub_mirxrors{i}(:, 15) = y - 0.5 .* param.len .* cos(ts) + 0.5 .* param.width .* sin(ts);
%     sub_mirxrors{i}(:, 16) = param.height - 0.5 .* param.len .* sin(tz);
end
    
end