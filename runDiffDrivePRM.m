%% a test script for simulator class
clear all;
close all;
load exampleMap % load example map into workspace

%% specify some parameters
numRobots = 5;
numSensors = 10;
sensorRange = 4;
showTraj = false;
initial_poses = 2*(rand(3,numRobots).*[1;1;pi] - [0.5;0.5;0]) + [2;2;0];
robotInfos = cell(1,numRobots);
for i = 1:numRobots
    t = "DiffDrive"; % differential drive dynamics
    R = 0.1; 
    L = 0.5;
    s = numSensors;
    r = sensorRange;
    show = showTraj;
    robotInfo = RobotInfo(t,R,L,s,r);
    robotInfos{i} = robotInfo;
end
swarmInfo = SwarmInfo(numRobots,robotInfos,initial_poses,false);
%% simple simulation
sim = DiffDrivePRMSimulation(map,swarmInfo);
for i = 1:1000
    sim = sim.step();
    axis([0 14 0 14])
    pause(0.02);
end