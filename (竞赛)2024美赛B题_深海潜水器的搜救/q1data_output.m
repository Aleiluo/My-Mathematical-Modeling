clc,clear;
clear global;
lon_range = [14.6, 22];
lat_range = [34.96, 40.43];
init_data(lon_range, lat_range);
%%
global depth_data;
global rho_data;
global current_data;
x_n = 100;
y_n = 50;
output_cur_rho_depth(x_n, y_n);

function output_cur_rho_depth(x_n, y_n)
    global rho_data;
    global current_data;
    global depth_data
    x_range(1) = max(rho_data.x_min, current_data.x_min);
    x_range(2) = min(rho_data.x_max, current_data.x_max);
    y_range(1) = max(rho_data.y_min, current_data.y_min);
    y_range(2) = min(rho_data.y_max, current_data.y_max);
    % 生成网格信息
    x = linspace(x_range(1), x_range(2), x_n);
    y = linspace(y_range(1), y_range(2), y_n);
    [X, Y] = meshgrid(x, y);
    % X, Y转回经纬度
    [LON, LAT] = Mercator2lonLat(X, Y);
    writematrix(LON, 'data2py/cur_rho.xlsx', 'Sheet', 'LON', 'WriteMode', 'overwritesheet');
    writematrix(LAT, 'data2py/cur_rho.xlsx', 'Sheet', 'LAT', 'WriteMode', 'overwritesheet');

    bottom = depth_data.F(X, Y);
    writematrix(bottom, 'data2py/cur_rho.xlsx', 'Sheet', "bottom", 'WriteMode', 'overwritesheet');
    for depth = [0, -999]
        rho = rho_data.F(X, Y, repmat(depth, size(X)));
        [u, v] = current_data.F(X, Y, repmat(depth, size(X)));
        writematrix(rho, 'data2py/cur_rho.xlsx', 'Sheet', "rho" + num2str(depth), 'WriteMode', 'overwritesheet');
        writematrix(u, 'data2py/cur_rho.xlsx', 'Sheet', "u" + num2str(depth), 'WriteMode', 'overwritesheet');
        writematrix(v, 'data2py/cur_rho.xlsx', 'Sheet', "v" + num2str(depth), 'WriteMode', 'overwritesheet');
    end
end