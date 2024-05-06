clc,clear;
close all;

warning('off');
T = readtable('./第11题附件/KS与浓度关系.xlsx');

x = table2array(T(2:9,2)) * 0.01;
y = table2array(T(2:end,3:end));


coef_values = {};
rsquare = [];
for i = 1:3
    for j = 1:size(y, 2)
        % 对数据进行模型拟合。
%         [fitresult1, gof1] = fit(x, y(8 * (i - 1) + 1:8 * i,j), 'fourier2');
        [fitresult1, gof1] = fit(x, y(8 * (i - 1) + 1:8 * i,j), 'poly1');
        coef_values{i,j} = coeffvalues(fitresult1);
        rsquare(i,j) = gof1.rsquare;
    end
end

coef_values = coef_values';
rsquare = rsquare';
lamb = [400:20:700]';
redcoef = coef_values(:,1);
yellowcoef = coef_values(:,2);
bluecoef = coef_values(:,3);
coef_result = table(redcoef, yellowcoef, bluecoef);
writetable(coef_result,'q1coef_poly1.xlsx');

redR2 = rsquare(:,1);
yellowR2 = rsquare(:,2);
blueR2 = rsquare(:,3);
rsquare_result = table(redR2, yellowR2, blueR2);
writetable(rsquare_result,'q1Rsquare_poly1.xlsx');







