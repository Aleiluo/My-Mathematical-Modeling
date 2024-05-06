function init_data(lon_range, lat_range)
    %% 海洋深度信息
    global depth_data;

    % 读取数据
    lon_elevation = ncread("data/Ionian_Sea_Depth.nc", "lon");
    lat_elevation = ncread("data/Ionian_Sea_Depth.nc", "lat");
    [depth_data.x, depth_data.y] = lonLat2Mercator(lon_elevation, lat_elevation);
    depth_data.x_max = max(depth_data.x);
    depth_data.x_min = min(depth_data.x);
    depth_data.y_max = max(depth_data.y);
    depth_data.y_min = min(depth_data.y);
    depth_data.elevation = double(ncread("data/Ionian_Sea_Depth.nc", "elevation"));
    depth_data.F = griddedInterpolant({depth_data.x, depth_data.y}, depth_data.elevation);

    %% 密度
    global rho_data;
%     lon_rho = double(ncread("data/ArgoData_2019.nc",'LONGITUDE'));
%     lat_rho = double(ncread("data/ArgoData_2019.nc",'LATITUDE'));
%     [rho_data.x, rho_data.y] = lonLat2Mercator(lon_rho', lat_rho');
%     rho_data.z = -double(ncread("data/ArgoData_2019.nc",'LEVEL'));
%     rho_data.rho = double(ncread("data/ArgoData_2019.nc",'PDEN'));
%     rho_data.rho = pagetranspose(rho_data.rho);
    lon_rho = double(ncread("data/Ionian_Sea_CTD.nc",'lon'));
    lat_rho = double(ncread("data/Ionian_Sea_CTD.nc",'lat'));
    [x, y] = lonLat2Mercator(lon_rho, lat_rho);
    rho_data.x = [];
    rho_data.y = [];
    rho_data.z = [];
    rho_data.rho = [];
    % 深度
    depth_salt = ncread("data/Ionian_Sea_CTD.nc",'z');
    % 盐度
    salinity_salt = ncread("data/Ionian_Sea_CTD.nc",'Salinity');
    % 每一个经纬度测了多少个盐度
    z_row_size = ncread("data/Ionian_Sea_CTD.nc",'z_row_size');
    cur_idx = 1;
    for i = 1:length(z_row_size)
        idx_span = cur_idx:cur_idx + z_row_size(i) - 1;
        cur_idx = cur_idx + z_row_size(i);
        rho_data.x(idx_span, 1) = x(i);
        rho_data.y(idx_span, 1) = y(i);
        rho_data.z(idx_span, 1) = -depth_salt(idx_span);
        rho_data.rho(idx_span, 1) = salinity_salt(idx_span) + 1e3;
    end
    rho_data.x_max = max(rho_data.x);
    rho_data.x_min = min(rho_data.x);
    rho_data.y_max = max(rho_data.y);
    rho_data.y_min = min(rho_data.y);
    [unique_x, unique_y, unique_z] = groupsummary([rho_data.x, rho_data.y, rho_data.z], rho_data.rho, @median);
    rho_data.F = scatteredInterpolant(unique_x(:, 1), unique_x(:, 2), unique_x(:, 3) , unique_y);
    rho_data.F.Method = 'linear';
    rho_data.x = unique_x;
    rho_data.y = unique_y;
    rho_data.z = unique_z;

    %% 洋流信息
    global current_data;
%     lon_cur = double(ncread("data/yomahaData_2019.nc",'LONGITUDE'));
%     lat_cur = double(ncread("data/yomahaData_2019.nc",'LATITUDE'));
%     [current_data.x, current_data.y] = lonLat2Mercator(lon_cur', lat_cur');
%     % u: 向东为正 v: 向北为正
%     current_data.u_surf = [ncread("data/yomahaData_2019.nc",'USF') * 10^-2]';
%     current_data.v_surf = [ncread("data/yomahaData_2019.nc",'VSF') * 10^-2]';
%     current_data.u_1000 = [ncread("data/yomahaData_2019.nc",'UDP') * 10^-2]';
%     current_data.v_1000 = [ncread("data/yomahaData_2019.nc",'VDP') * 10^-2]';
    

    load('data/yomaha07.mat');
    error_val = [-999.9999,-99.9999,-999.9999,-999.9999,-999.9999,-999.9999,-99.9999,-999.9999,-999.9999];
    yomaha07 = yomaha07(sum(abs(yomaha07 - error_val) < 0.1, 2) == 0, :);
    yomaha07(:, 3) = -yomaha07(:, 3);
    yomaha07(:,[9, 10]) = yomaha07(:,[8, 9]);
    yomaha07(:, 8) = 0;
    yomaha07_tmp = yomaha07(:, 6:10);
    yomaha07 = yomaha07(:, 1:5);
    yomaha07 = [yomaha07; yomaha07_tmp];
    save_idx = yomaha07(:, 1) >= lon_range(1) & yomaha07(:, 1) <= lon_range(2) & ...
               yomaha07(:, 2) >= lat_range(1) & yomaha07(:, 2) <= lat_range(2);
    yomaha07 = yomaha07(save_idx, :);
    yomaha07(:, 4:5) = yomaha07(:, 4:5) * 10^-2;
    [current_data.x, current_data.y] = lonLat2Mercator(yomaha07(:, 1), yomaha07(:, 2));
    current_data.z = yomaha07(:, 3);
    current_data.u = yomaha07(:, 4);
    current_data.v = yomaha07(:, 5);
    current_data.x_max = max(current_data.x);
    current_data.x_min = min(current_data.x);
    current_data.y_max = max(current_data.y);
    current_data.y_min = min(current_data.y);
    current_data.Fu = scatteredInterpolant(current_data.x, current_data.y, current_data.z , current_data.u);
    current_data.Fv = scatteredInterpolant(current_data.x, current_data.y, current_data.z , current_data.v);
    current_data.Fu.Method = 'natural';
    current_data.Fv.Method = 'natural';
    current_data.F = @(x, y, z) get_cur(x, y, z);
end

function [u,v] = get_cur(x, y, z)
    global current_data;
    if z > -1000
        u = current_data.Fu(x, y, z);
        v = current_data.Fv(x, y, z);
    else
        u = current_data.Fu(x, y, -1000);
        v = current_data.Fv(x, y, -1000);
    end
end