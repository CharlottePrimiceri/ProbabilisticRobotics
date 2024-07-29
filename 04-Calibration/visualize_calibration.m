%put also there the ground truth trajectory?
%show the uncalibrated and calibrated odometry? 
close all
clear
clc
#prova
#import 2d geometry utils
source "../tools/utilities/geometry_helpers_2d.m"
#source "../04-Calibration/calibrated_odometry.m"
disp('loading the matrix');

% Read dataset.... transform then into function
#function []=reaf_file(path)
file = fopen('dataset.txt', 'r');
if file == -1
    error('Failed to open the file.');
end
#don't include the commented rows
for i = 1:8
    fgetl(file);
end
#reds text into a cell array of strings, each line is a cell
dataset = textscan(file, '%s', 'Delimiter', '\n', 'MultipleDelimsAsOne', 1);

#dataset = textscan(file, '%s: %f %s: %d %d %s: %f %f %f %s: %f %f %f', 'Delimiter', ' ');
lines = dataset{1};
n_lines = size(lines)(1);
disp(n_lines);


%Z=load('/home/charlotte/Documents/ProbabilisticRobotics/04-Calibration/dataset.txt');
%addpath('/home/charlotte/Documents/ProbabilisticRobotics/04-Calibration');

#compute the ground truth trajectory
#TrueTrajectory=odometry_trajectory(Z(:,1:3));

%disp('2D position of sensor w.r.t. base link');
%S = function1();
%disp(S);
%pause(1);

%disp('kinematic parameters');
%P = function2();
%disp(P);
