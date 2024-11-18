close all
clear
clc
#import 2d geometry utils
source "../tools/utilities/geometry_helpers_2d.m"
disp('loading the matrix');
path = 'dataset.txt';

# Read dataset
function odometry=read_odometry(path)
        file = fopen(path, 'r');

        if file == -1
            error('Failed to open the file.');
        end

        #reads text into a cell array of strings, each line is a cell
        dataset = textscan(file, '%s', 'Delimiter', '\n', 'MultipleDelimsAsOne', 1);
        lines = dataset{1};
        n_lines = size(lines)(1);
        #the incremental encoder values are stored in a uint32 variable with max range of:
        overflow_inc_enc = 4294967295;
        previous_inc_t = 4294859756;
        inc_enc_update = 0;
        #odometry matrix, without first 8 lines
        odometry = zeros(n_lines-8, 9);
        #from 9 so it doesn't include the first 8 rows
        for i = 9:n_lines 
            line = lines{i};
            line = textscan(line, 'time: %f ticks: %f %f model_pose: %f %f %f tracker_pose: %f %f %f', 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
            tick_inc = line{3};
            current_inc_t = tick_inc;
            delta = current_inc_t - previous_inc_t;
            overflow_delta = overflow_inc_enc + delta;
            # if those two conditions occurs there is overflow
            if (current_inc_t < previous_inc_t && overflow_delta < (previous_inc_t - current_inc_t))
               inc_enc_update = overflow_delta;
               tick_inc = inc_enc_update;
               #display('overflow')
            else
               inc_enc_update = delta;
               tick_inc = inc_enc_update;
               #display('non overflow')
            endif
            previous_inc_t = current_inc_t;
            odometry(i-8,:)=[cell2mat(line(1:2)), tick_inc, cell2mat(line(4:9))];
        endfor    
endfunction

U = read_odometry(path);
display(size(U));

function dataset_time=reset_time(U)

        # because of the floating point, is better to reinitialize the time to have more precise increment value of time
        #in matlab if we compute the eps(number_a), the error that can be computed between number_a and the minum computable consecutive one number_b,
        #eps(1.6e+09) = 2.38e-07 
        #if there is an increment of the last two digits in 1668091584.821040869 (example of our dataset) it would be lost.
        time_column = U(:,1);
        time_update = 0;
        previous_time = time_column(1,1);
        n =size(time_column,1);
        time_column(1,1) = time_update;
        for i = 1:(n-1)
            current_time = time_column(i+1,1);
            time_increment = current_time - previous_time;
            time_update += time_increment;
            time_column(i+1,1) = time_update;
            previous_time = current_time;
        endfor
 
        U(:,1)= time_column;
        dataset_time = U;

endfunction

U = reset_time(U);
%display(U(:,1))

function pose_T=predictFront_Tractor_Tricycle(traction_incremental_ticks,
                                                 initial_state, steering_ticks, max_enc_values, kin_parameters)
        ticks_to_meters=kin_parameters(2);
        traction_max = max_enc_values(2);
        axis_lenght = kin_parameters(3);
        steer_offset=kin_parameters(4);
        steer_max=max_enc_values(1);
        ticks_to_radians = kin_parameters(1);
        theta = initial_state(3);
        
        # traction_incremental_ticks are the increments of the encoder computed in the dataset function
        # (ticks_to_meters / traction_max) is the value of meters corresponds to one single tick
        traction_front = traction_incremental_ticks * (ticks_to_meters / (traction_max));
        
        # (ticks_to_radians *2*pi / steer_max) is the value of radians (converted from revolution to
        # radians with a factor 2pi)  corresponds to one single tick in case of positive angles
        if steering_ticks > (steer_max/2)
           # considering negative angles
           steer_angle = - (ticks_to_radians * (steer_max - steering_ticks)*2*pi/(steer_max)) + steer_offset;
        else
           # positive angles
           steer_angle = (ticks_to_radians * steering_ticks *pi *2 / (steer_max)) + steer_offset;
        endif

        # drawing the model of the tricycle we obtain that relationship for the 
        # translational and rotational displacements
        # The equations for the transition function are obtained by integrating the 
        # translational velocity and the rotational velocity in the time interval
        # v = delta_s/delta_T
        # then apply Euler Integration x_t = x_(t-1) + v * delta_t * cos(theta_(t-1)+psi_(t-1))
        dx = traction_front *cos(theta + steer_angle);
        dy = traction_front *sin(theta + steer_angle);
        dth = traction_front * (sin(steer_angle))/axis_lenght;
        x = initial_state(1) + dx;
        y = initial_state(2) + dy;
        th = initial_state(3) + dth;
        pose_T = [x; y; th];

endfunction

function laser_pose = laser(kin_parameters, front_odometry)
        axis_lenght = kin_parameters(3);
        laser_base = [kin_parameters(5), kin_parameters(6), kin_parameters(7)];
        laser_base_neg = [-kin_parameters(5), -kin_parameters(6), -kin_parameters(7)];
        #display(laser_base)
        T_laser_base_neg = v2t(laser_base_neg);
        T_laser_base = v2t(laser_base);
        #laser_pose = [];
        #n = size(front_odometry, 2);
        n = length(front_odometry(1,:));
        laser_pose = zeros(3, n);
        #display(n)
        for i = 1:n
        #display('front_odometry')
        #display(front_odometry)
            rear_x = front_odometry(1, i) - axis_lenght*cos(front_odometry(3,i));
            rear_y = front_odometry(2, i) - axis_lenght*sin(front_odometry(3,i));
            rear_pose = [rear_x; rear_y; front_odometry(3,i)];
            T_rear = v2t(rear_pose);
            T_laser = T_rear * T_laser_base;
            T_laser_n = T_laser_base_neg * T_laser;
            laser_pose_v = t2v(T_laser_n);
            #display("laser_pose")
            #laser_pose = [laser_pose; laser_pose_v];
            laser_pose(:,i)=[laser_pose_v];
            #display(laser_pose)
        endfor
        #display(laser_pose)
        laser_pose = laser_pose';
endfunction

function Z = robot_config_f(initial_state, max_enc_values, U_b, kin_parameters, steering)
        inc_enc_column = U_b(:,3);
        abs_enc_column = U_b(:,2);
        n = size(inc_enc_column, 1);
        Z=zeros(3, n);
        for i = 1:n
            if i == 1
                # initialize the first to zero because we need to consider the angle at the previous time step
                # and the same for the rotational displacement
                # no need to do that for the increment because it is already zero
                steering_ticks = steering; # modified this for the batches
                Z(1:3,i) = predictFront_Tractor_Tricycle(inc_enc_column(i),
                                                 initial_state, steering_ticks, max_enc_values, kin_parameters);
                initial_state = Z(1:3,i);
            else
                steering_ticks = abs_enc_column(i-1);
                Z(1:3,i) = predictFront_Tractor_Tricycle(inc_enc_column(i),
                                                 initial_state, steering_ticks, max_enc_values, kin_parameters);
                initial_state = Z(1:3,i);
            endif
        endfor
        
endfunction

#### GIVEN PARAMETERS
kinematic_parameters = [0.1; 0.0106141; 1.4; 0; 1.5; 0; 0];
max_enc_values = [8192 5000];
# psi is omitted 
initial_state = [1.4; 0; 0];
indices = (1:2434)';
num_kin = length(kinematic_parameters);
dataset_size = size(U, 1);
steer_v = 0;

####################### GROUND TRUTH ##############################
% plot(U(:,7), U(:,8),-'o'); 
% axis([-5 4 -4 2]);
% title ("Ground Truth xy trajectory of Laser wrt baselink");
% pause(5);
% plot(indices, U(:,9),-'o');
% axis([-2 2434 -5 5]);
% title ("Ground Truth theta values of Laser wrt baselink");
% pause(5)
 
% ## Ground Truth Uncalibrated Odometry of the Front Wheel
% plot(U(:,4), U(:,5), -'o');
% axis([-5 25 -14 2]);
% title ("Ground Truth Uncalibrated Odometry of the Front Wheel");
% pause(5);

####################### UNCALIBRATED POSE ###########################

% ## Predicted Uncalibrated Odometry of Front Wheel with initial guess of parameters 
% T = robot_config_f(initial_state, max_enc_values, U, kinematic_parameters, steer_v);
% plot(T(1,:),T(2,:),-'o');
% axis([-5 25 -13 2]);
% title ("Initial Predicted Uncalibrated xy trajectory of Front Wheel");
% pause(5);

% ## Predicted Uncalibrated Odometry of Laser Pose w.r.t the baselink with initial guess of parameters 
% pose_laser = laser(kinematic_parameters, T);
% plot(pose_laser(:,1),pose_laser(:,2),-'o');
% axis([-5 25 -13 2]);
% title ("Initial Predicted Uncalibrated xy trajectory of Laser wrt baselink");
% pause(5);

#Normalize Angles to do difference
function angle_difference = angle_normalization(theta_1, theta_2)
    norm_theta_1 = mod(theta_1 + pi, 2 * pi) - pi;
    norm_theta_2 = mod(theta_2 + pi, 2 * pi) - pi;
    difference = norm_theta_1 - norm_theta_2;
    angle_difference = mod(difference + pi, 2 * pi) - pi;
endfunction


############ LEAST SQUARES FOR BATCHES ############
function kinematic_parameters = LeastSquares(kinematic_parameters, max_enc_values, initial_state, epsilon, n_iteration, dataset_size, num_kin, steer_v, U)

        #Initialize Kernel
        kernel_threshold = 1; 
        final_threshold = 1e-2; 
        threshold_decay = (kernel_threshold  - final_threshold) / n_iteration;

        for iteration = 1:n_iteration
            current_threshold = kernel_threshold - iteration * threshold_decay;
            current_threshold = max(current_threshold, final_threshold);

            T_pred_all = robot_config_f(initial_state, max_enc_values, U, kinematic_parameters, steer_v);
            laser_pred_all = laser(kinematic_parameters, T_pred_all);


            laser_all_kin_plus = [];
            laser_all_kin_minus = [];
            
            # add perturbation
            perturbation = zeros(num_kin, 1);
            for i = 1:num_kin
                perturbation(i) = epsilon;
                
                # positive perturbation
                front_plus = robot_config_f(initial_state, max_enc_values, U, kinematic_parameters + perturbation, steer_v);
                laser_plus = laser(kinematic_parameters + perturbation, front_plus);
                # negative perturbation 
                front_minus = robot_config_f(initial_state, max_enc_values, U, kinematic_parameters - perturbation, steer_v);
                laser_minus = laser(kinematic_parameters - perturbation, front_minus);
                # remember to reset to zero this vector so to perturb only one parameter
                perturbation(i) = 0;
                # stack the predicted dataset pertubed by one kinematic parameter at a time
                laser_all_kin_plus = [laser_all_kin_plus;laser_plus];
                laser_all_kin_minus = [laser_all_kin_minus;laser_minus];
                #display(size(laser_all_kin_plus));
            endfor    
        
            delta_x = zeros(num_kin);
            H = zeros(num_kin, num_kin);
            b = zeros(num_kin, 1);
            c = 0;

            for i=1:dataset_size

                # Compute Error
                error = zeros(3,1);
                pred = laser_pred_all(i, :); 
                meas = U(i, 7:9);
                error(1:2, 1) = pred(:, 1:2) - meas(:, 1:2);
                error(3, 1) = angle_normalization(pred(:, 3), meas(:, 3)); 

                # Compute Jacobian 
                Jacobian = zeros(3, 7);
                first_k_laser_value = 1;
                for k=1:num_kin
                    last_k_laser_value = (first_k_laser_value + dataset_size)-1; 

                    # Prepare the perturbated dataset for each of the current kinematic parameter considered
                    laser_all_kin_plus_k = laser_all_kin_plus(first_k_laser_value:last_k_laser_value, :); 
                    laser_plus_i = laser_all_kin_plus_k(i, :);

                    laser_all_kin_minus_k = laser_all_kin_minus(first_k_laser_value:last_k_laser_value, :);
                    laser_minus_i = laser_all_kin_minus_k(i, :);

                    Jacobian(1:2, k) = (laser_plus_i(:, 1:2)) - (laser_minus_i(:, 1:2));
                    Jacobian(3, k) = angle_normalization(laser_plus_i(:, 3), laser_minus_i(:, 3));

                    first_k_laser_value += dataset_size; 

                endfor
                # scale the gradient
                Jacobian = Jacobian * (0.5/epsilon);

                # Robust Estimator
                if (c > current_threshold)
                    error *= sqrt(current_threshold / c);
                    c = current_threshold;
                endif

                H += (Jacobian' * Jacobian);
                b += (Jacobian' * error);
                c += (error' * error);

            endfor

            delta_x = -(pinv(H))*b; 
           
            kinematic_parameters += delta_x;


            display('Error')
            display(c)
            display(iteration)
            display('Calibrated Kinematic Parameters')
            display(kinematic_parameters)

            calibrated_front_pose = robot_config_f(initial_state, max_enc_values, U, kinematic_parameters, steer_v);
            calibrated_pose_laser = laser(kinematic_parameters, calibrated_front_pose);
            #xy
            plot(calibrated_pose_laser(:,1),calibrated_pose_laser(:,2),-'o');
            axis([-5 4 -4 2]);
            title ("Calibrate- xy trajectory of Laser wrt baselink");    
            pause(2)
        endfor       
endfunction

####### Calibrated 2D Laser Pose Trajectory + Calibrated kinematic parameters after 70 iterations of LS ########
epsilon = 1e-04;
dataset_size = 2434; 
n_iteration=70;
kinematic_parameters = LeastSquares(kinematic_parameters, max_enc_values, initial_state, epsilon, n_iteration, dataset_size, num_kin, steer_v, U);


% ####################### CALIBRATED POSE ###########################
% calibrated_front_pose_f = robot_config_f(initial_state, max_enc_values, U, kinematic_parameters, steer_v);
% calibrated_pose_laser_f = laser(kinematic_parameters, calibrated_front_pose_f);

% plot(calibrated_pose_laser_f(:,1),calibrated_pose_laser_f(:,2),-'o');
% axis([-5 4 -4 2]);
% title ("Converging xy trajectory of Laser Pose wrt baselink");
% pause(10);

% plot(indices, calibrated_pose_laser_f(:,3),-'o');
% axis([-2 2434 -5 5]);
% title ("Converging theta values of Laser Pose wrt baselink");
% pause(10);


