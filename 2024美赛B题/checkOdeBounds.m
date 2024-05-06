function [value,isterminal,direction] = checkOdeBounds(t, xyz_v)
    global depth_data;
    bottom_z = depth_data.F(xyz_v(1), xyz_v(2));
    value(1, 1) = xyz_v(3) - bottom_z;
    value(2, 1) = xyz_v(3) - 0;
    isterminal = [1; 1];
    direction = [-1; 1];
end