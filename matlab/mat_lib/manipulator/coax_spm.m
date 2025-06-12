classdef coax_spm < handle
    properties
        a1
        a2
        b
        v
        w
        n
        u
        i_range
        v_origin
        n_origin
        eta
        w_fpk
        v_fpk
        actuator_origin
        actuator_direction
    end

    methods
        function obj = coax_spm(a1, a2, b)
            obj.a1 = a1;
            obj.a2 = a2;
            obj.b = b;
            obj.v = zeros(3, 3);
            obj.w = zeros(3, 3);
            obj.n = zeros(3, 1);
            obj.u = [0; 0; -1];
            obj.i_range = 0:2;
            obj.v_origin = zeros(3, 3);
            for i = obj.i_range
                obj.v_origin(:, i+1) = obj.v_i_origin(i);
            end
            obj.n_origin = [0; 0; 1];
            obj.eta = zeros(1,3);
            for i = obj.i_range
                obj.eta(i+1) = obj.eta_i(i);
            end
            obj.w_fpk = zeros(3, 3);
            obj.v_fpk = zeros(3, 3);
            obj.actuator_origin = -pi;
            obj.actuator_direction = -1;
        end

        function eta = eta_i(~, i)
            eta = 2 * i * pi / 3;
        end

        function w = w_i(obj, in_i, i)
            e_i = obj.eta_i(i);
            w = [cos(e_i - in_i) * sin(obj.a1);
                 sin(e_i - in_i) * sin(obj.a1);
                 -cos(obj.a1)];
        end

        function v = v_i_origin(obj, i)
            e_i = obj.eta_i(i);
            v = [-sin(e_i) * sin(obj.b);
                  cos(e_i) * sin(obj.b);
                  cos(obj.b)];
        end

        function A = A_i_ipk(obj, v_i, i)
            e_i = obj.eta_i(i);
            A = -v_i(1) * cos(e_i) * sin(obj.a1) - ...
                v_i(2) * sin(e_i) * sin(obj.a1) - ...
                v_i(3) * cos(obj.a1) - cos(obj.a2);
        end

        function B = B_i_ipk(obj, v_i, i)
            e_i = obj.eta_i(i);
            B = v_i(1) * sin(e_i) * sin(obj.a1) - ...
                v_i(2) * cos(e_i) * sin(obj.a1);
        end

        function C = C_i_ipk(obj, v_i, i)
            e_i = obj.eta_i(i);
            C = v_i(1) * cos(e_i) * sin(obj.a1) + ...
                v_i(2) * sin(e_i) * sin(obj.a1) - ...
                v_i(3) * cos(obj.a1) - cos(obj.a2);
        end

        function input_angle = solve_ipk(obj, r)
            for i = obj.i_range
                obj.v(:, i+1) = r * obj.v_origin(:, i+1);
            end
            obj.n = r * obj.n_origin;
            T = zeros(3,2);
            for i = obj.i_range
                A_ = obj.A_i_ipk(obj.v(:, i+1), i);
                B_= obj.B_i_ipk(obj.v(:, i+1), i);
                C_ = obj.C_i_ipk(obj.v(:, i+1), i);
                T(i+1,:) = real(solve_quadratic(A_, 2 * B_, C_));
            end
            T_solution = T(:,1);
            input_angle = real(2 * atan(T_solution));
            for i = obj.i_range
                obj.w(:, i+1) = obj.w_i(input_angle(i+1), i);
            end
            obj.verify_position();
        end

        function obj = verify_position(obj)
            for i = obj.i_range
                if isclose(angle_between(obj.u, obj.w(:, i+1)), obj.a1) == false
                    error('Rotation invalid! parameter a1 is incorrect');
                elseif isclose(angle_between(obj.w(:, i+1), obj.v(:, i+1)), obj.a2) == false
                    error('Rotation invalid! parameter a2 is incorrect');
                elseif isclose(angle_between(obj.v(:, i+1), obj.n), obj.b) == false
                    error('Rotation invalid! parameter b is incorrect');
                end
            end
        end

        function w_actuator = solve_ivk(obj, ypr, w_ypr)
            w_xyz = ypr_to_xyz_velocity(w_ypr, ypr);
            r_ypr = rotz(ypr(1)) * roty(ypr(2)) * rotx(ypr(3));
            actuators = solve_ipk(obj, r_ypr);
            A = zeros(3,3);
            B = zeros(3,3);
            for i = obj.i_range
                A(i+1, :) = cross(obj.w(:, i+1), obj.v(:, i+1))';
                B(i+1, i+1) = dot(cross(obj.u, obj.w(:, i+1)), obj.v(:, i+1));
            end
            J = B\A;
            w_actuator = J * w_xyz;
        end

        function r = fpk_system(obj, v_out)
            r = zeros(9,1);
            vec = reshape(v_out, [3, 3])';
            w_vec = obj.w_fpk;
            
            for i = 1:3
                r(i) = dot(w_vec(i, :), vec(i, :));
            end

            r(4) = dot(vec(1, :), vec(2, :)) - cos(2 * pi / 3);
            r(5) = dot(vec(2, :), vec(3, :)) - cos(2 * pi / 3);
            r(6) = dot(vec(3, :), vec(1, :)) - cos(2 * pi / 3);
            r(7) = norm(vec(1, :))^2 - 1;
            r(8) = norm(vec(2, :))^2 - 1;
            r(9) = norm(vec(3, :))^2 - 1;
        end

        function ypr = solve_fpk(obj, input_angles)
            yaw_offset = wrap_rad(obj.actuator_direction * ...
                                  (input_angles(1) + obj.actuator_origin));
            offset_input_angles = input_angles + yaw_offset;
            init_v = [1, 1, 1, -1, -1, 1, 1, -1, 1];
            for i = obj.i_range
                obj.w_fpk(i+1, :) = obj.w_i(offset_input_angles(i+1), i)';
            end
            options = optimoptions('fsolve', 'Display', 'off', 'Algorithm', 'levenberg-marquardt');
            v_out = fsolve(@(x) obj.fpk_system(x), init_v, options);
            obj.v_fpk = reshape(v_out, [3, 3])';
            for i = 1:3
                obj.v_fpk(i, :) = (rotz(yaw_offset) * obj.v_fpk(i, :)')';
            end
            ortho = get_orthonormals(obj);
            ypr = unwind_ypr(ortho);
        end

        % get orthonormal vectors (xyz) relative to the body frame
        function ortho = get_orthonormals(obj)
            y_v = obj.v_fpk(1,:);
            z_v = unit_vector(cross(obj.v_fpk(1,:), obj.v_fpk(2,:)));
            % minus puts the x axis in the positive x direction
            x_v = -unit_vector(cross(z_v, y_v));
            ortho = [x_v; y_v; z_v];
        end
    end
end