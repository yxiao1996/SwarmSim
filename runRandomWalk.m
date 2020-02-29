%% a test script for simulator class
clear all;
close all;
load exampleMap % load example map into workspace

%% specify some parameters
p.numRobots = 10;
p.numSensors = 10;
p.sensorRange = 4;
p.showTraj = false;
% generate some initial positions
p.poses = 4*(rand(3,p.numRobots).*[1;1;pi] - [0.5;0.5;0]) + [9;9;0];

%% simple simulation
sim = RandomWalkSimulation(map,p);
for i = 1:100
    sim = sim.step();
    axis([0 14 0 14])
    pause(0.05);
end