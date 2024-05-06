function param = calDNI(D, ST, param)
% 输入
% D：为以春分（3月21日）作为第0天起算的天数
% ST：当地时间（24小时制），单位：小时
    
    % 基础参数设定
    G0 = 1.366;
    H = 3000;
    phi = 39.4 * pi / 180;
    a = 0.4237 - 0.00821 * (6 - H / 1e3)^2;
    b = 0.5055 + 0.00595 * (6.5 - H / 1e3)^2;
    c = 0.2711 + 0.01858 * (2.5 - H / 1e3)^2;

    w = pi * (ST - 12) / 12;
    delta = asin(sin(2 * pi * D / 365) * sin(2 * pi * 23.45 / 360));
    param.alpha_s = asin(cos(delta) * cos(phi) * cos(w) + sin(delta) * sin(phi));
    param.gamma_s = asin(cos(delta) * sin(w) / cos(param.alpha_s));
%     param.gamma_s = acos((sin(delta) - sin(param.alpha_s) * sin(phi)) / (cos(param.alpha_s) * cos(phi)));
%     param.gamma_s = real(param.gamma_s);
    % 原点指向太阳的向量
    param.light_vec = [-cos(param.alpha_s) * cos(param.gamma_s - pi / 2), cos(param.alpha_s) * sin(param.gamma_s - pi / 2), sin(param.alpha_s)];
    param.DNI = G0 * (a + b * exp(-c / sin(param.alpha_s)));
end