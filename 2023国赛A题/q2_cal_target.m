function mirrors = q2_cal_target(mirrors, param)

% 遮蔽效率
mirrors(:, 12) = 0.9;

% 截断效率
mirrors(:, 13) = 0.9;

% 余弦效率
mirrors(:, 14) = mirrors(:, 9:11) * param.light_vec';

% 大气透射效率
x = mirrors(:, 1);
y = mirrors(:, 2);
z = mirrors(:, 3);
dHR = sqrt((0 - x).^2 + (0 - y).^2 + (param.Hz - z).^2);
mirrors(:, 15) = 0.99321 - 0.0001176 .* dHR + 1.97 .* 1e-8 .* dHR.^2;

% 镜面反射效率
mirrors(:, 16) = 0.92;

% 光学效率
mirrors(:, 17) = prod(mirrors(:, 12:16), 2);

% 计算面积与效率的乘积
mirrors(:, 18) = mirrors(:, 4) .* mirrors(:, 5) .* mirrors(:, 17);


end
    



