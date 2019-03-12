%% a test script for simulator class
clear all;
close all;
load exampleMap % load example map into workspace

%% specify some parameters
numRobots = 5;
numSensors = 4;
sensorRange = 4;
dynamics = "OmniDir";
showTraj = false;
initial_poses = (rand(3,numRobots).*[0.25;0.25;0]) + [11;1.8;0];
robotInfos = cell(1,numRobots);
for i = 1:numRobots
    % differential drive dynamics
    t = dynamics;
    R = 0.1; 
    L = 0.5;
    s = numSensors;
    r = sensorRange;
    show = showTraj;
    robotInfo = RobotInfo(t,R,L,s,r);
    robotInfos{i} = robotInfo;
end
swarmInfo = SwarmInfo(numRobots,robotInfos,initial_poses,false);
%% virtual structure simulation
sim = VirtualStructureSimulation(map,swarmInfo);
for i = 1:50
    sim = sim.step();
    axis([0 14 0 14])
    pause(0.02);
end