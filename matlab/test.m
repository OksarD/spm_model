% params

a1 = pi/4; % alpha 1
a2 = pi/2; % alpha 2
b = pi/2; % beta

spm = coax_spm(a1,a2,b);

% inverse velocity

ypr = [0, 0, 0]
w_ypr = [1,1,1]

actuator_velocity = solve_ivk(spm, ypr, w_ypr)

% forward position

act_pos = [0,0,0]

ypr_out = solve_fpk(spm, act_pos)