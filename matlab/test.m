ypr = [pi/4 pi/6 pi/12]

w_ypr = [1,1,1];

v_fpk = [-0.592, 0.775, 0.224; ...
    -0.235, -0.918, 0.321; ...
    0.826, 0.143, -0.5450];

w_xyz = ypr_to_xyz_velocity(w_ypr, ypr)
ypr_fpk = unwind_ypr(v_fpk)

