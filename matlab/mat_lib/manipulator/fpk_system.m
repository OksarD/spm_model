classdef fpk_system
    properties
        v_fpk
        w_fpk
    end

    methods
        % quadratic system to solve fpk problem. Has eight solutions so the initial guess must be chosen carefully to ensure the correct solution is found
        function r_out = fpk_system(v_out)
            % Initialize residuals
            r = zeros(9);
            % rename variables for ease of use
            v0_x = v_out(1);
            v0_y = v_out(2);
            v0_z = v_out(3);
            v1_x = v_out(4);
            v1_y = v_out(5);
            v1_z = v_out(6);
            v2_x = v_out(7);
            v2_y = v_out(8);
            v2_z = v_out(9);
            % w[1][2] is w_0_y (first number is ith w vector, second number is xyz selection)
            w = w_fpk;
            % angle to intermediate joint constraint
            r(1) = w(1,1)*v0_x + w(1,2)*v0_y + w(1,3)*v0_z;
            r(2) = w(2,1)*v1_x + w(2,2)*v1_y + w(2,3)*v1_z;
            r(3) = w(3,1)*v2_x + w(3,2)*v2_y + w(3,3)*v2_z;
            % vector angular constraint
            r(4) = v0_x*v1_x + v0_y*v1_y + v0_z*v1_z - cos(2*pi/3);
            r(5) = v1_x*v2_x + v1_y*v2_y + v1_z*v2_z - cos(2*pi/3);
            r(6) = v2_x*v0_x + v2_y*v0_y + v2_z*v0_z - cos(2*pi/3);
            % unit vector magnitude constraint
            r(7) = v0_x^2 + v0_y^2 + v0_z^2 - 1;
            r(8) = v1_x^2 + v1_y^2 + v1_z^2 - 1;
            r(9) = v2_x^2 + v2_y^2 + v2_z^2 - 1;
            r_out = r;
        end
    end
end