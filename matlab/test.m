a1 = pi/4; % alpha 1
a2 = pi/2; % alpha 2
b = pi/2; % beta

spm = coax_spm(a1,a2,b);

ypr = [pi/4, pi/6, pi/12]
w_ypr = [1,1,1]

w_xyz = ypr_to_xyz_velocity(w_ypr, ypr)
r_ypr = rotz(ypr(1)) * roty(ypr(2)) * rotx(ypr(3))

input_angle = solve_ipk(spm, r_ypr)

