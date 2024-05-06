function s = cal_shadow_area(A, len, width)
% 寻找最外层的点，并且求其覆盖的阴影面积
% A：点集
% k：阴影点集的类型
% k = 1:右上, 2:左上, 3:左下, 4:右下
s = 0;

for k = 1:4
    if isempty(A{k})
        continue;
    end
    if k == 1
        % 上下镜像，左右镜像
        A{k} = A{k} * -1;
    elseif k == 2
        % 上下镜像
        A{k}(:, 2) = A{k}(:, 2) * -1;
    % k == 3 情况无需镜像

    elseif k == 4
        % 左右镜像
        A{k}(:, 1) = A{k}(:, 1) * -1;
    end
    % 统一平移至原点
    A{k}(:, 1) = A{k}(:, 1) + width / 2;
    A{k}(:, 2) = A{k}(:, 2) + len / 2;

    % 计算阴影面积
    s = s + single_shadow(A{k});

end

end

function s = single_shadow(A)
% A对横坐标排序
A = sortrows(A, 1);

% 从右往左找
A_size = size(A, 1);
B = [];
y_max = -inf;
for i = A_size:-1:1
    if A(i, 2) >= y_max
        y_max = A(i, 2);
        B = [B; A(i, :)];
    end
end

% 求阴影面积(最外层点与x,y的正半轴围成的多个矩形区域)
B_size = size(B, 1);
s = B(1, 1) * B(1, 2);
for i = 2:B_size
    s = s + (B(i, 1) - B(i - 1, 1)) * B(i, 2);
end

end