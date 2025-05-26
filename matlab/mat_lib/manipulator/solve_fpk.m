% numerically solves fpk problem and returns the rotation matrix
function ypr = solve_fpk(input_angles)
        % decouple yaw by subtracting first angle
        yaw_offset = wrap_rad(self.actuator_direction*(input_angles[0] - self.actuator_origin))
        offset_input_angles = input_angles + yaw_offset
        init_v = [1,1,1,-1,-1,1,1,-1,1] % initial vector guess which corresponds to l-l-l configuration
        % nonlinear canonical system of equations
        self.w_fpk = np.array([self.w_i(offset_input_angles[i], i) for i in self.i_range])
        v_out = optimize.fsolve(func=self.fpk_system, x0=init_v)
        self.v_fpk = np.array([v_out[i*3:i*3+3] for i in self.i_range])
        self.v_fpk = [R_z(yaw_offset) @ self.v_fpk[i] for i in self.i_range]
        print("v_fpk")
        print(self.v_fpk)
        ypr = self.unwind_ypr(self.v_fpk)
        return ypr