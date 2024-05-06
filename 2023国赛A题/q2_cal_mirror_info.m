function mirrors = q2_cal_mirror_info(mirrors, param)
% 问题二改写的计算定日镜信息函数

% 使用向量化计算
x = mirrors(:, 1);
y = mirrors(:, 2);
z = mirrors(:, 3);

% 反射光线计算
sublen = length(x);
r = [0 - x, 0 - y, param.Hz - z];
r = r ./ sqrt(r(:, 1).^2 + r(:, 2).^2 + r(:, 3).^2);
a_theta2 = sqrt((1 + param.light_vec * r') ./ 2)';
n = (param.light_vec + r) ./ (2 .* a_theta2);  

% 计算俯仰角(注意，真的俯仰角要用90度减去该值)
tz = pi / 2 - asin(n(:,3));
mirrors(:, 7) = tz;
% 计算方位角
ts = atan(n(:, 2) ./ n(:, 1));
mirrors(:, 8) = ts;

% 记录法向量
mirrors(:, 9:11) = n;
end
