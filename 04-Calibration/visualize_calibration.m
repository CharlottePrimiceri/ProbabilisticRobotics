%show the uncalibrated and calibrated odometry? 
close all
clear
clc
#import 2d geometry utils
source "../tools/utilities/geometry_helpers_2d.m"
source "../04-Calibration/odometry_trajectory.m"
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
###ctrl k u ctrl k c commentare scommentae 
#+str{variabile_nome}+
U = read_odometry(path);
display(size(U))

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

U = reset_time(U)
display(U(:,1))

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

function laser_pose = laser(kin_parameters, front_odometry, laser_base)
        axis_lenght = kin_parameters(3);
        laser_base = [laser_base(1); laser_base(2); laser_base(3)];
        #display(laser_base)
        T_laser_base = v2t(laser_base);
        laser_pose_vector = [];
        n = size(front_odometry, 2);
        display(n)
        for i = 1:n
            rear_x = front_odometry(1, i) + axis_lenght*cos(front_odometry(3,i));
            rear_y = front_odometry(2, i) + axis_lenght*sin(front_odometry(3,i));
            rear_pose = [rear_x; rear_y; front_odometry(3,i)];
            T_rear = v2t(rear_pose);
            T_laser = T_rear * T_laser_base;
            laser_pose_v = t2v(T_laser);
            #display(laser_pose_v)
            laser_pose = [laser_pose_vector; laser_pose_v];
        endfor
        #display(laser_pose_v)
        
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
                                                 initial_state, steering_ticks, max_enc_values, kin_parameters)
                initial_state = Z(1:3,i);
            else
                steering_ticks = abs_enc_column(i-1);
                Z(1:3,i) = predictFront_Tractor_Tricycle(inc_enc_column(i),
                                                 initial_state, steering_ticks, max_enc_values, kin_parameters)
                initial_state = Z(1:3,i);
            endif
        endfor
        
endfunction


kinematic_parameters = [0.1 0.0106141 1.4 0];
laser_baselink = [1.5 0 0];
max_enc_values = [8192 5000];
#initial guess [x y theta psi]
initial_state = [0; 0; 0];
inc_enc_value =U(:,3);
abs_enc_value=U(:,2);

########### Plot the Uncalibrated Odometry of the Front Wheel ###########
T = robot_config_f(initial_state, max_enc_values, U, kinematic_parameters);
#plot(T(1,:),T(2,:),-'o');
# chosen those value of axis just beacuse i've known the true trajectory from the python file provided
#axis([-5 25 -13 2]);
#pause(10);

########### Compute the laser pose w.r.t the reference frame  ###########
pose_laser = laser(kinematic_parameters, T, laser_baselink)

#plot ground truth of the laser pose trajectory 
#plot(U(:,7), U(:,8),-'o' ); 
# chosen those value of axis just because i've known the true trajectory from the python file provided
#axis([-5 4 -4 2]);
#pause(10);

#compute the uncalibrated odometry
#plot(U(:,4), U(:,5), 'b-', 'linewidth', 2);
#pause(10);


%disp('2D position of sensor w.r.t. base link');
%S = function1();
%disp(S);
%pause(1);

%disp('kinematic parameters');
%P = function2();
%disp(P);