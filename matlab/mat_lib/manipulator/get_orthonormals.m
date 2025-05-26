% get orthonormal vectors (xyz) relative to the body frame
function ortho = get_orthonormals(v)
    y_v = v(1,:);
    z_v = unit_vector(cross(v(1,:), v(2,:)));
    % minus puts the x axis in the positive x direction
    x_v = -unit_vector(cross(z_v, y_v));
    ortho = [x_v; y_v; z_v];
end