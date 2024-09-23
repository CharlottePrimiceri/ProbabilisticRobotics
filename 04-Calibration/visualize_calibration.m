%show the uncalibrated and calibrated odometry? 
close all
clear
clc
#import 2d geometry utils
source "../tools/utilities/geometry_helpers_2d.m"
source "../04-Calibration/odometry_trajectory.m"
disp('loading the matrix');
path = 'dataset.txt';
# Read dataset.... transform then into function

#initial guess [x y theta psi]
initial_state = [0 0 0 0];


function odometry=read_odometry(path)
        file = fopen(path, 'r');

        if file == -1
            error('Failed to open the file.');
        end

        #reads text into a cell array of strings, each line is a cell
        dataset = textscan(file, '%s', 'Delimiter', '\n', 'MultipleDelimsAsOne', 1);
        lines = dataset{1};
        n_lines = size(lines)(1);
        #disp(n_lines);

        #odometry matrix, without first 8 lines
        odometry = zeros(n_lines-8, 9);
        #from 9 so it doesn't include the first 8 rows
        for i = 9:n_lines
            line = lines{i};
            line = textscan(line, 'time: %f ticks: %f %f model_pose: %f %f %f tracker_pose: %f %f %f', 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
            odometry(i-8,:) = cell2mat(line(1:9)); #here modified 4:6
        end    
endfunction

function new_dataset=eliminate_overflow(U)

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

#the incremental encoder values are stored in a uint32 variable with max range of:
overflow_inc_enc = 4294967295;

#if the previous value of tick is greater than the next one but cannot be considered 
#as a backward motionthere is overflow, so write the change in
#ticks as joints_max_enc_values - previous_tick_value + new_tick_value
#otherwhise maintain the actual difference
inc_enc_column = U(:,3);
inc_enc_update = 0;
previous_inc_t = inc_enc_column(1,1);
for i = 1:(n-1)
    current_inc_t = inc_enc_column(i+1,1);
    delta = current_inc_t - previous_inc_t;
    # if those two conditions occurs there is overflow
    if current_inc_t < previous_inc_t && delta > (overflow_inc_enc/2)
       inc_enc_update = overflow_inc_enc - delta;
       inc_enc_column(i+1,1) = inc_enc_update;
    else
        inc_enc_update = delta;
        inc_enc_column(i+1,1) = inc_enc_update;
    endif
    previous_inc_t = current_inc_t;    
endfor

endfunction


U = read_odometry(path);
laser_pose = U(:,7:9);
#disp(inc_enc_column);
%disp(size(time_column,1))
#compute the ground truth trajectory
#T = odometry_trajectory(U(:,4:6));
#disp('ground truth');
#hold on;
plot(laser_pose(:,1),laser_pose(:,2),-'o' ); 
# chosen those value of axis just beacuse i've known the true trajectory 
# from the python file provided
axis([-5 4 -4 2]);
#'r-', 'linewidth', 2);
pause(10);


#compute the uncalibrated odometry
#uncalibrated_odometry = odometry_trajectory(U(:,1:3));
#disp('Uncalibrated Odometry');
#hold on;
#plot(uncalibrated_odometry(:,1), uncalibrated_odometry(:,2), 'b-', 'linewidth', 2);
#pause(10);


%disp('2D position of sensor w.r.t. base link');
%S = function1();
%disp(S);
%pause(1);

%disp('kinematic parameters');
%P = function2();
%disp(P);
