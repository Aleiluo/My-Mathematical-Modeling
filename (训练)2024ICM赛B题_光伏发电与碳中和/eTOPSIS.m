clc,clear
T = readtable('各区块数据V2(保留).xlsx');
T = removevars(T, ["number","OPTA","CDD","HDD","LANDC","Center_Latitude",...
    "Center_Longitude","province","build_cost","single_Q","single_benifit"]);
A = table2array(T(:,1:13));
A = fillmissing(A,'linear',1);
category = {2, 1, 1, 1, 1, 2, 1, 4, 2, 2, 2, 2, 1;
            0, 0, 0, 0, 0, 0, 0, [15,35],0, 0, 0, 0, 0};
%%
[score,RANK] = MyTOPSIS(A, category, 4);
% 得分的排名写到文件中
T_out = array2table([score,RANK], 'VariableNames', {'score', 'Rank'});

% 保存表格到CSV文件
writetable(T_out, '排名信息.xlsx');