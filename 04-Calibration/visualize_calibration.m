%show the uncalibrated and calibrated odometry? 
close all
clear
clc
#import 2d geometry utils
source "../tools/utilities/geometry_helpers_2d.m"
#source "../04-Calibration/odometry_trajectory.m"
#source "../04-Calibration/tricycle.m"
disp('loading the matrix');
path = 'dataset.txt';
# Read dataset.... transform then into function

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

#+str{variabile_nome}+
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
display(U(:,1));

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

function Z = robot_config_f(initial_state, max_enc_values, U, kin_parameters)
        inc_enc_column = U(:,3);
        abs_enc_column = U(:,2);
        n = size(inc_enc_column, 1);
        Z=zeros(3, n);
        for i = 1:n
            if i == 1
                # initialize the first to zero because we need to consider the angle at the previous time step
                # and the same for the rotational displacement
                # no need to do that for the increment because it is already zero
                steering_ticks = 0;
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


kinematic_parameters = [0.1; 0.0106141; 1.4; 0; 1.5; 0; 0];
max_enc_values = [8192 5000];
initial_state = [0; 0; 0];
inc_enc_value =U(:,3);
abs_enc_value=U(:,2);
########### Plot the Uncalibrated Odometry of the Front Wheel ###########
#T = robot_config_f(initial_state, max_enc_values, U, kinematic_parameters);
#plot(T(1,:),T(2,:),-'o');
#axis([-5 25 -13 2]);
#pause(10);
#save myfile.mat T
#T = load('myfile.mat');
#display(T)
########### Display the laser pose w.r.t the reference frame  ###########
#pose_laser = laser(kinematic_parameters, T, laser_baselink)

########### Plot the Ground Truth 2D Laser Pose Trajectory ###########
#plot(U(:,7), U(:,8),-'o' ); 
#axis([-5 4 -4 2]);
#pause(10);

########### Plot the Ground Truth Uncalibrated Odometry of the Front Wheel ###########
#display(U(:, 4:6))
#plot(U(:,4), U(:,5), 'b-', 'linewidth', 2);
#pause(10);

########### Display Calibrated Kinematic Parameters ###########
#display('Calibrated Kinematic Parameters')
% calibrated_kin_parameter, chi = LS(U, kinematic_parameters, T, laser_baselink)
% display(calibrated_kin_parameter);
% calibrated_firstKin_parameters = calibrated_kin_parameter(1:4);
% calibrate_laser_baselink = calibrated_kin_parameter (5:7);

########### Plot Calibrated 2D Laser Pose Trajectory ###########
#display('Calibrated 2D Laser Pose Trajectory')
#calibrated_pose_laser = laser(calibrated_firstKin_parameters, T, calibrate_laser_baselink)
#plot(calibrated_pose_laser(1,:),calibrated_pose_laser(2,:),-'o');
#pause(10);

#Normalize Angles to do difference
function angle_difference = angles_difference(theta_1, theta_2)
    norm_theta_first = mod(theta_1 + pi, 2 * pi) - pi;
    norm_theta_second = mod(theta_2 + pi, 2 * pi) - pi;
    difference = norm_theta_1 - norm_theta_2;
    angle_difference = mod(difference + pi, 2 * pi) - pi;
endfunction

#try not to use separated functions and put all together in the main

dataset_size = size(U,1);
epsilon = 1e-04;
n_kin_parameters = size(kinematic_parameters);
# compute the laser pose with the initial guess of kinematic parameters
T = robot_config_f(initial_state, max_enc_values, U, kinematic_parameters);
laser = laser(kinematic_parameters, T);
# add perturbation
perturbation = zeros(n_kin_parameters, 1);
for i = 1:n_kin_parameters
    perturbation(i) = epsilon; 
    front_plus = robot_config_f(initial_state, max_enc_values, U, kinematic_parameters + epsilon);
    front_minus = robot_config_f(initial_state, max_enc_values, U, kinematic_parameters + epsilon);
    laser_plus = laser(kin_parameters, front_odometry);
    laser_minus = laser(kin_parameters, front_odometry);

endfor

% function [f_p, f_m, l_p, l_m] = addPerturbation(initial_state, max_enc_values, U, kinn_parameters)
%         for i=1:n_kin_par
%             epsilon = zeros(7,1);
%             epsilon(i)=1e-4;
%             front_plus = robot_config_f(initial_state, max_enc_values, U, kinn_parameters + epsilon(1:4));
%             front_minus = robot_config_f(initial_state, max_enc_values, U, kinn_parameters - epsilon(1:4));
%             laser_plus = laser(kin_parameters, front_odometry, laser_base);
%             laser_minus = laser(kin_parameters, front_odometry, laser_base);
%         endfor    

%  endfunction

% function [e, J] = errorAndJacobian(measures, kine_parameters, pred, initial_state, max_enc_values, laser_baselink)
%         full_kin_parameter = [kine_parameters; laser_baselink];
%         meas = measures(:, 7:9);
%         #pred = laser(kine_parameters, front_i, laser_baselink);
%         #display('pred')
%         #display(pred)
%         #pred_xy = pred(1:3, :);
%         n_kin_par = length(full_kin_parameter);
%         #pred_t = pred_xy';
%         #display('measures')
%         #display(meas)
%         #display(pred_xy )
%         meas_t = meas'; 
%         e = zeros(3, 1);
%         e(1:2, 1) = pred(1:2) - meas_t(1:2); 
%         e(3, 1) = angle_difference(pred(3), meas_t(3));
%         #e(3) = ;
%         #e = e_v;
%         #display('errorrrr')
%         #display(e)
%         # size J depends on x
%         #front_plus = robot_config_f(initial_state, max_enc_values, meas, kine_parameters + epsilon(1:4));
%         #front_minus = robot_config_f(initial_state, max_enc_values, meas, kine_parameters - epsilon(1:4));
        
%         J = zeros(3, n_kin_par);
%         for i=1:n_kin_par
%             epsilon = zeros(7,1);
%             epsilon(i)=1e-4;
%             #display('epsilon')
%             #display(epsilon)
%             #display(epsilon(1:4))
%             #display('kine_paramters')
%             #display(kine_parameters)
%             #display(kine_parameters+epsilon(1:4))
%             #front_plus = robot_config_f(initial_state, max_enc_values, meas, kine_parameters + epsilon(1:4));
%             #front_minus = robot_config_f(initial_state, max_enc_values, meas, kine_parameters - epsilon(1:4));
%             #primo = laser(kine_parameters + epsilon(1:4), front_plus, laser_baselink + epsilon(5:7));
%             #display('primo')
%             #display(primo)
%             #display('secondo')
%             #secondo = laser(kine_parameters - epsilon(1:4), front_minus, laser_baselink - epsilon(5:7));
%             #display(secondo)
%             # primo (i)
%             # secondo (i)
%             difference = primo(1:2) - secondo(1:2);
%             #display('difference')
%             #display(difference)
%             #display(size(primo))
%             #display(size(secondo))
%             J(1:2,i) = difference;
%             J(3, i) = angle_difference(primo(3), secondo(3)); 
%             #display(J(:,i))
%             #J(:,i) = laser(kine_parameters + epsilon(1:4), T, laser_baselink + epsilon(5:7)) - laser(kine_parameters - epsilon(1:4), T, laser_baselink + epsilon(5:7));
%         endfor
%         #display(J)
%         J = (J * 0.5)/(1e-4);
%         #J = (J/((2e-3)*epsilon'));
% endfunction        

% function [kin_par, c] = LS(U_meas, k_parameters, las, initial_state, max_enc_values, laser_baselink_parameters)
%         full_kin_parameters = [k_parameters; laser_baselink_parameters];
%         n_kin_par = length(full_kin_parameters);
%         delta_x = zeros(n_kin_par);
%         H = zeros(n_kin_par, n_kin_par);
%         b = zeros(n_kin_par, 1);
%         c = 0;
%         for i=1:(size(U_meas,1))
%             # prima diemnsione di U
%             #display('size (U,1)')
%             #display(size(U,1))
%             #display(T(:, i))
%             #display('T(:, i)')
%             [e, J] = errorAndJacobian(U_meas(i, :), k_parameters, las(:,i), initial_state, max_enc_values, laser_baselink_parameters);
%             H += J' * J;
%             b += J' * e;
%             c += e' * e;
%         endfor

%         delta_x = -(pinv(H))*b;
%         full_kin_parameters += delta_x;
%         kin_par = full_kin_parameters;
% endfunction

% #display(length(T(:,)))
% ########### Display Calibrated Kinematic Parameters ###########
% #display('Calibrated Kinematic Parameters')
% function calibrated_kin_parameter = roundls(initial_state, max_enc_values, U, kinematic_parameters, laser_baselink)
%         ki_par = kinematic_parameters;
%         las_b = laser_baselink;
%         for i=1:6
%             T_n = robot_config_f(initial_state, max_enc_values, U, ki_par);
%             laser_p = laser(ki_par, T_n, las_b);
%             [parameters, chichi] = LS(U, ki_par, laser_p, initial_state, max_enc_values, las_b);
%             ki_par = parameters(1:4);
%             las_b = parameters(5:7);
%             display(parameters);
%         endfor
%         calibrated_kin_parameter = [ki_par; las_b];
%         display(calibrated_kin_parameter);
% endfunction

% #calibrated_kin_parameter = LS(U, kinematic_parameters, T, laser_baselink);
% final_result = roundls(initial_state, max_enc_values, U, kinematic_parameters, laser_baselink);
% #display(final_result)
% calibrated_firstKin_parameters = final_result(1:4);
% calibrate_laser_baselink = final_result(5:7);

% T_calibrated = robot_config_f(initial_state, max_enc_values, U, calibrated_firstKin_parameters);
% #calibrated_kin_parameter = LS(U, calibrated_firstKin_parameters, T_calibrated, calibrate_laser_baselink);
% #final_result = roundls(U, calibrated_firstKin_parameters, T_calibrated, calibrate_laser_baselink);
% #display(final_result)
% #ultimo_grafico = laser(calibrated_firstKin_parameters, T_calibrated, calibrate_laser_baselink)
% plot(T_calibrated(1,:), T_calibrated(2,:),-'o');
% #axis([-5 4 -4 2]);
% axis([-5 25 -13 2]);
% pause(10);
% ########### Plot Calibrated 2D Laser Pose Trajectory ###########
% #display('Calibrated 2D Laser Pose Trajectory')
% #calibrated_pose_laser = laser(calibrated_firstKin_parameters, T, calibrate_laser_baselink);
% #plot(calibrated_pose_laser(1,:),calibrated_pose_laser(2,:),-'o');
% #axis([-5 25 -13 2]);
% #pause(10);