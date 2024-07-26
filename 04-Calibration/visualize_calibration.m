%put also there the ground truth trajectory?
%show the uncalibrated and calibrated odometry? 
close all
clear
clc

#import 2d geometry utils
source "../tools/utilities/geometry_helpers_2d.m"

disp('loading the matrix');
% Read the file content
fid = fopen('dataset.txt');
if fid == -1
    error('Failed to open the file.');
end
% Initialize an empty array to store the model_pose values
model_pose_values = [];

% Read the file line by line
while ~feof(fid)
    line = fgetl(fid);
    
    % Use a regular expression to extract the model_pose values
    pattern = 'model_pose:\s*([^\s]+)\s*([^\s]+)\s*([^\s]+)';
    tokens = regexp(line, pattern, 'tokens');
    
    % If model_pose values are found, convert them to numbers and store them

    if ~isempty(tokens)
        values = str2double(tokens{1});
        model_pose_values = [model_pose_values; values]; % Append to the array
    end
end
% Close the file
fclose(fid);

% Display the extracted model_pose values
disp('Model Pose Values:');
disp(model_pose_values);
#a
%Z=load('/home/charlotte/Documents/ProbabilisticRobotics/04-Calibration/dataset.txt');
%addpath('/home/charlotte/Documents/ProbabilisticRobotics/04-Calibration');

%compute the ground truth trajectory
%TrueTrajectory=compute_odometry_trajectory(Z(:,1:3));

%disp('2D position of sensor w.r.t. base link');
%S = function1();
%disp(S);
%pause(1);

%disp('kinematic parameters');
%P = function2();
%disp(P);
