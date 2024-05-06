clear
warning('off')

tranNodes = readtable('第二问促动器与主索节点信息.xlsx');
workingEdge = readtable('第二问工作区内主索长度.xlsx');

Nodenums = size(tranNodes,1);
Edgenums = size(workingEdge,1);
c1 = 561.7228;
c2 = 300.8443;
k1 = 1 / c1;
k2 = -c2;
% 计算lc ls
Lc = sqrt((tranNodes.ux - tranNodes.lx).^2 + ...
          (tranNodes.uy - tranNodes.ly).^2 + ...
          (tranNodes.uz - tranNodes.lz).^2);

Ls = sqrt((tranNodes.x - tranNodes.ux).^2 + ...
          (tranNodes.y - tranNodes.uy).^2 + ...
          (tranNodes.z - tranNodes.uz).^2);

% 优化变量初始值
initial_vars = [tranNodes.x; tranNodes.y; tranNodes.z; zeros(Nodenums, 1)];
%initial_vars = [tranNodes.x; tranNodes.y; k1 .* tranNodes.x.^2 + k1 .* tranNodes.y.^2 + k2; zeros(Nodenums, 1)];
result = initial_vars;
% 定义变量上下界
lb = [-305 * ones(3 * Nodenums, 1); -0.6 * ones(Nodenums, 1)];
ub = [155 * ones(3 * Nodenums, 1); 0.6 * ones(Nodenums, 1)];
% 定义目标函数
objective = @(vars) obj(vars, Nodenums);
% 定义非线性约束函数
nonlcon = @(vars) cons(vars, Lc, Ls, tranNodes, workingEdge);

% 求解器选项
options = optimoptions(@fmincon);
options.Algorithm = "interior-point";
options.MaxIterations = Inf;
options.MaxFunctionEvaluations = Inf;
options.Display = 'iter-detailed';
% options.SubproblemAlgorithm = 'cg';
options.HessianApproximation = 'lbfgs';
options.SpecifyConstraintGradient = true;
options.SpecifyObjectiveGradient = false;
options.EnableFeasibilityMode = true;
options.HonorBounds = true;
options.ConstraintTolerance = 1e-12;
options.StepTolerance = 1e-12;
options.OptimalityTolerance = 1e-12;
% 梯度检查与函数值检查
options.CheckGradients = false;
option.FunValCheck = 'off';

[result, fval] = fmincon(objective, result, [], [], [], [], lb, ub, nonlcon, options);

% 保存结果到文件
% writematrix(result,'q2result.txt');

function [f] = obj(vars, Nodenums)
    % 目标函数
    % f:目标函数值
    % grad:梯度
    c1 = 561.7228;
    c2 = 300.8443;
    k1 = 1 / c1;
    k2 = -c2;
    wn = reshape(vars(1:Nodenums*3), Nodenums, 3);

    dh = wn(:, 3) - k1 .* wn(:, 1).^2 - k1 .* wn(:, 2).^2 - k2;
    f = sum(dh.^2);

%     % 对各自变量(3460个)求偏导
%     grad = [-2 .* k1 .* dh .* wn(:,1); ...
%             -2 .* k1 .* dh .* wn(:,2); ...
%             2 .* k1 .* dh; ...
%             zeros(Nodenums, 1)];
end

function [c, ceq, gradc, gradceq] = cons(vars, Lc, Ls, tranNodes, workingEdge)
    % c(x) <= 0
    % ceq(x) = 0
    % 设定常量
    Nodenums = size(tranNodes, 1);
    Edgenums = size(workingEdge, 1);
    % 提取变量
    wn = reshape(vars(1:Nodenums*3), Nodenums, 3);
    lamb = vars(Nodenums*3+1:end);
 
    % 等式约束
    xpuls = (tranNodes.ux - tranNodes.lx) .* lamb ./ Lc + tranNodes.ux;
    ypuls = (tranNodes.uy - tranNodes.ly) .* lamb ./ Lc + tranNodes.uy;
    zpuls = (tranNodes.uz - tranNodes.lz) .* lamb ./ Lc + tranNodes.uz;
    ceq = (wn(:,1) - xpuls).^2 + ...
          (wn(:,2) - ypuls).^2 + ...
          (wn(:,3) - zpuls).^2 - Ls.^2;

    % 等式梯度(每一列一个目标函数，每一行一个自变量的偏导)
    dxpuls_dlamb = (tranNodes.ux - tranNodes.lx) ./ Lc;
    dypuls_dlamb = (tranNodes.uy - tranNodes.ly) ./ Lc;
    dzpuls_dlamb = (tranNodes.uz - tranNodes.lz) ./ Lc;
    gradceq = [diag(2 .* (wn(:,1) - xpuls)); ...
               diag(2 .* (wn(:,2) - ypuls)); ...
               diag(2 .* (wn(:,3) - zpuls)); ...
               diag(-2.*(wn(:,1) - xpuls).*dxpuls_dlamb + ...
                    -2.*(wn(:,2) - ypuls).*dypuls_dlamb + ...
                    -2.*(wn(:,3) - zpuls).*dzpuls_dlamb)];

    % 不等约束
    node1 = workingEdge.node1+1;
    node2 = workingEdge.node2+1;
    len = workingEdge.len;
    c = [(0.9993 * len).^2 - ...
         ((wn(node1,1) - wn(node2,1)).^2 + ...
         (wn(node1,2) - wn(node2,2)).^2 + ...
         (wn(node1,3) - wn(node2,3)).^2);
         ...
         ((wn(node1,1) - wn(node2,1)).^2 + ...
         (wn(node1,2) - wn(node2,2)).^2 + ...
         (wn(node1,3) - wn(node2,3)).^2) - ...
         (1.0007 * len).^2];

    % 不等式梯度
    gradc = zeros(length(vars), size(c,1));
    for k = 1:Edgenums
        i = node1(k);
        j = node2(k);
        addposi = [i, Nodenums + i, 2 * Nodenums + i];
        addposj = [j, Nodenums + j, 2 * Nodenums + j];
        gradc(addposi, k) = [-2*(wn(i,1)-wn(j,1)); -2*(wn(i,2)-wn(j,2)); -2*(wn(i,3)-wn(j,3))];
        gradc(addposj, k) = [2*(wn(i,1)-wn(j,1)); 2*(wn(i,2)-wn(j,2)); 2*(wn(i,3)-wn(j,3))];
        gradc(addposi, k + Edgenums) = [2*(wn(i,1)-wn(j,1)), 2*(wn(i,2)-wn(j,2)), 2*(wn(i,3)-wn(j,3))];
        gradc(addposj, k + Edgenums) = [-2*(wn(i,1)-wn(j,1)), -2*(wn(i,2)-wn(j,2)), -2*(wn(i,3)-wn(j,3))];
    end

end



