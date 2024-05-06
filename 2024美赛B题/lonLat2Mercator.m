function [x, y]  = lonLat2Mercator(lon, lat)
    x = lon .* 20037508.34 ./ 180;
    ly = log(tan((90 + lat) .* pi / 360)) ./ (pi / 180);
    y =  ly .* 20037508.34 ./ 180;
end