function w_xyz = ypr_to_xyz_velocity(w_ypr, ypr)
    % https://physics.stackexchange.com/questions/492906/application-of-angular-velocity-to-euler-angles
    w_yaw = w_ypr(1);
    w_pitch = w_ypr(2);
    w_roll = w_ypr(3);
    yaw = ypr(1);
    pitch = ypr(2);
    x = [1; 0; 0];
    y = [0; 1; 0];
    z = [0; 0; 1];
    E_yaw = rotz(yaw);
    E_pitch = E_yaw*roty(pitch);
    w_xyz = (w_yaw*z + w_pitch*(E_yaw*y) + w_roll*(E_pitch*x));
end