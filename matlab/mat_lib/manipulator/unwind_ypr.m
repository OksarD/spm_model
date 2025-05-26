% unwinds ypr angles from three-vector (v) platform representation. Angular range is -180 <= a < 180
function ypr = unwind_ypr(v)
    % normal, pitch and roll axes in the body frame
    ortho = get_orthonormals(v);
    roll_ax = ortho(1,:);
    pitch_ax = ortho(2,:);
    normal_z = [0,0,1];
    % using angle_between will not provide angles above 180, so the roll/pitch unwind range is limited to +/- 90 degrees
    roll_projected_vector = unit_vector(project_to_plane(roll_ax, normal_z));
    roll = pi/2 - angle_between(roll_projected_vector, pitch_ax);
    pitch_ax = rotaxis(roll_ax, -roll) * pitch_ax.';
    pitch = -(pi/2 - angle_between(normal_z, roll_ax));
    roll_ax = rotaxis(pitch_ax, -pitch) * roll_ax.';
    % use trig values to convert unit vector to angle to avoid 180 degree constraint
    yaw = atan2(roll_ax(2), roll_ax(1));
    ypr = [yaw, pitch, roll];
end