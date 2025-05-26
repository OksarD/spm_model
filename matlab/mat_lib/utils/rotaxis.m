function mat = rotaxis(axis, a)
    u_x = axis(1);
    u_y = axis(2);
    u_z = axis(3);
    r = 1-cos(a);
    s = sin(a);
    c = cos(a);
    mat = [(u_x^2)*r + c, u_x*u_y*r - u_z*s, u_x*u_z*r + u_y*s; ...
                     u_x*u_y*r + u_z*s, (u_y^2)*r + c, u_y*u_z*r - u_x*s; ...
                     u_x*u_z*r - u_y*s, u_y*u_z*r + u_x*s, (u_z^2)*r + c];
end