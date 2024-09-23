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

#if the previous value of tick is greater than the next one there is overflow, so write the change in
#ticks as joints_max_enc_values - previous_tick_value + new_tick_value
#otherwhise leave the normal difference

#because of the floating point, is better to reinitialize the time to have more precise increment value of time
#in matlab if we compute the eps(number_a), the error that can be computed between number_a and the minum computable consecutive one number_b,
#eps(1.6e+09) = 2.38e-07 
#if there is an increment of the last two digits in 1668091584.821040869 (example of our dataset) it would be lost.
time = 0;

#the incremental encoder values are stored in a uint32 variable with max range of:
overflow_inc_enc = 4294967295;
inc_enc = 0;

endfunction


U = read_odometry(path);
disp(U)
#compute the ground truth trajectory
#T = odometry_trajectory(U(:,4:6));
#disp('ground truth');
#hold on;
#plot(T(:,1),T(:,2), 'r-', 'linewidth', 2);
#pause(10);


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
