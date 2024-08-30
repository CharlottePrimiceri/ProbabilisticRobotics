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
        odometry = zeros(n_lines-8, 6);
        #from 9 so it doesn't include the first 8 rows
        for i = 9:n_lines
            line = lines{i};
            line = textscan(line, 'time: %f ticks: %d %d model_pose: %f %f %f tracker_pose: %f %f %f', 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
            odometry(i-8,:) = cell2mat(line(4:9)); #here modified 4:6
        end    
endfunction

U = read_odometry(path);
disp(U)
#compute the ground truth trajectory
T = odometry_trajectory(U(:,4:6));
disp('ground truth');
hold on;
plot(T(:,1),T(:,2), 'r-', 'linewidth', 2);
pause(10);


#compute the uncalibrated odometry
uncalibrated_odometry = odometry_trajectory(U(:,1:3));
disp('Uncalibrated Odometry');
hold on;
plot(uncalibrated_odometry(:,1), uncalibrated_odometry(:,2), 'b-', 'linewidth', 2);
pause(10);
%disp('2D position of sensor w.r.t. base link');
%S = function1();
%disp(S);
%pause(1);

%disp('kinematic parameters');
%P = function2();
%disp(P);
