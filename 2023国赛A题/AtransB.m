function [p14, p_mid] = AtransB(m1, m2, light_vec)
% m2的5个镜场坐标系转为m1的坐标
% 光照向量转为m1的坐标
% 做投影工作
Eh = m1(7);
Ah = m1(8);
T = [-sin(Eh), -sin(Ah) .* cos(Eh), cos(Ah) .* cos(Eh); ...
     cos(Eh),  -sin(Ah) .* sin(Eh), cos(Ah) .* sin(Eh); ...
     0,        cos(Ah),             sin(Ah)];

% 获取m2的5个镜场坐标
points = reshape(m2(12:26), 3, 5);

P_new = T' * (points - m1(1:3)');
P_new = P_new';

Vh = T' * light_vec';

proj_point = zeros(size(P_new));
proj_point(:, 1) = (Vh(3) .* P_new(:, 1) - Vh(1) .* P_new(:, 3)) ./ Vh(3);
proj_point(:, 2) = (Vh(3) .* P_new(:, 2) - Vh(2) .* P_new(:, 3)) ./ Vh(3);
proj_point(:, 3) = 0;


p14 = proj_point(1:4, :);
p_mid = proj_point(5, :);


end