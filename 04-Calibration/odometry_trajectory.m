#source "../tools/utilities/geometry_helpers_2d.m"

function A = v2t(v)
    % This function converts a vector to a transformation matrix
    A = [cos(v(3)), -sin(v(3)), v(1);
         sin(v(3)),  cos(v(3)), v(2);
         0,          0,        1];
end

function v = t2v(T)
    v = [T(1,3), T(2,3), atan2(T(2,1), T(1,1))];
end

#U: matrix contains odometry, it provides the incremental changes in the robot's position and orientation at each time step
# from gaussian newton method x = x + deltax
# deltax is our perturbation
# x are our kinematic parameters to be estimated by minimizing the error of the laser pose prediction
function [e, J] = errorAndJacobian(U, kinematic_parameters, T, laser_baselink)
        full_kin_parameters = [kinematic_parameters ; laser_baselink ];
        meas = U(:, 7:8);
        pred = laser(kinematic_parameters, T, laser_baselink);
        #pred = pred(1:2, :);
        n_kin_par = len(full_kinematic_parameters);
        pred_t = pred';
        e = pred - meas; 
        # size J depends on x
        J = zeros(2, n_kin_par);
        for i=1:n_kin_par
            epsilon = zeros(7,1);
            epsilon(i)=1e-3;
            J(:,i) = laser(kinematic_parameters + epsilon(1:4), T, laser_baselink + epsilon(5:7)) - laser(kinematic_parameters - epsilon(1:4), T, laser_baselink + epsilon(5:7));
        endfor
        J/=2e-3;
endfunction        

function [kin_par, c] = LS(U, kinematic_parameters, T, laser_baselink)
        full_kin_parameters = [kinematic_parameters ; laser_baselink ];
        n_kin_par = len(full_kinematic_parameters);
        H = zeros(n_kin_par, n_kin_par);
        b = zeros(n_kin_par, 1);
        c = 0;
        for i=1:(size(U,1))
            [e, J] = errorAndJacobian(U, kinematic_parameters, T(1:2, i), laser_baselink);
            H += J' * J;
            b += J' * e;
            c += e' * e;
        endfor
        delta_x = -(pinv(H))/b;
        kin_par += delta_x;
endfunction



