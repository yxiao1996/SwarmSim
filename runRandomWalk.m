%% a test script for simulator class
clear all;
close all;
load exampleMap % load example map into workspace

%% specify some parameters
p.numRobots = 3;
numSensors = 1;
sensorRange = 4;

p.infos = cell(1,p.numRobots);
% generate some initial positions
p.poses = 4*(rand(3,p.numRobots).*[1;1;pi] - [0.5;0.5;0]) + [9;9;0];
p.showTraj = false;

for i = 1:p.numRobots
    t = "DiffDrive"; % differential drive dynamics
    R = 0.1; 
    L = 0.5;
    s = numSensors;
    r = sensorRange;
    show = p.showTraj;
    robotInfo = RobotInfo(t,R,L,s,r);
    p.infos{i} = robotInfo;
end
%swarmInfo = SwarmInfo(numRobots,robotInfos,initial_poses,false);

%% simple simulation
sim = RandomWalkSimulation(map,p);
for i = 1:100
    sim = sim.step();
    axis([0 14 0 14])
    pause(0.05);
end