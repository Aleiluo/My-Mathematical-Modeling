clc,clear;


%%
x = [210 57	27	15	160	80 400	325	100];
y = [500 2000	7000	11000	400	900 500	500	600];

% x_range = linspace(10, 400, 1000);
% y_range = spline(x, y, x_range);
% plot(x_range, y_range);

spline(x, y, 400)

%% TOPSIS气动
T1 = readtable('data/q2dataV2.xlsx','Sheet','AUV');
T2 = readtable('data/q2dataV2.xlsx','Sheet','声纳');
T3 = readtable('data/q2dataV2.xlsx','Sheet','洋流');
T4 = readtable('data/q2dataV2.xlsx','Sheet','水深');
D1 = table2array(T1(:, 2:end));
D2 = table2array(T2(:, 2:end));
D3 = table2array(T3(:, 2:end));
D4 = table2array(T4(:, 2:end));

c1 = {2, 1, 1, 1; 0, 0, 0, 0};
c2 = {2, 1, 1, 1, 2, 1; 0, 0, 0, 0, 0, 0};
c3 = {2, 1, 1, 2, 2, 2; 0, 0, 0, 0, 0, 0};
c4 = {1, 2, 1, 2, 1; 0, 0, 0, 0, 0};

[score1,RANK1] = MyTOPSIS(D1, c1, 2);
[score2,RANK2] = MyTOPSIS(D2, c2, 2);
[score3,RANK3] = MyTOPSIS(D3, c3, 2);
[score4,RANK4] = MyTOPSIS(D4, c4, 2);

%% 数据导出
T1 = readtable('data/q2dataV2.xlsx','Sheet','AUV');
T2 = readtable('data/q2dataV2.xlsx','Sheet','声纳');
T3 = readtable('data/q2dataV2.xlsx','Sheet','洋流');
T4 = readtable('data/q2dataV2.xlsx','Sheet','水深');

T1 = addvars(T1, score1, RANK1);
T1 = sortrows(T1, "RANK1");
writetable(T1,'data/q2output.xlsx', 'Sheet', 'AUV');

T2 = addvars(T2, score2, RANK2);
T2 = sortrows(T2, "RANK2");
writetable(T2,'data/q2output.xlsx', 'Sheet', '声纳');

T3 = addvars(T3, score3, RANK3);
T3 = sortrows(T3, "RANK3");
writetable(T3,'data/q2output.xlsx', 'Sheet', '洋流');

T4 = addvars(T4, score4, RANK4);
T4 = sortrows(T4, "RANK4");
writetable(T4,'data/q2output.xlsx', 'Sheet', '水深');


