%% a test script for the world model
clear all;
close all;
load exampleMap % load example map into workspace
%% specify some parameters
numRobots = 3;
numSensors = 25;
sensorRange = 4;
showTraj = false;
% generate some initial positions
initial_poses = 4*(rand(3,numRobots).*[1;1;pi] - [0.5;0.5;0]) + [9;9;0];
w = world(numRobots,numSensors,sensorRange,showTraj,initial_poses); 
readings = w.readSensors();
w.visualize(readings);

dTheta = pi/64;

for idx = 1:100
    % Update the environment and poses
    poses = w.get_poses();
    poses(3,:) = poses(3,:)  + dTheta;
    w = w.update_poses(poses);
    readings = w.readSensors();
    %w.visualize();
    w.visualize(readings);
    pause(0.05);
end
