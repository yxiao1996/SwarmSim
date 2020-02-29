%% a test script for simulator class
clear all;
close all;
% generate map for the simulation
size = 15;
resolution = 10;
numObstacles = 2;
space = 5;
%p = zeros(size*resolution);
map_gen = MapGenerate(size,size,space,resolution);
[p,map_gen] = map_gen.addBounds(2);
for i = 1:numObstacles
    [p,map_gen] = map_gen.addRandomObstacle(1.5,0.5);
end
map = binaryOccupancyMap(p,resolution);

%% specify some parameters
numRobots = 5;
numSensors = 4;
sensorRange = 4;
dynamics = "OmniDir";
showTraj = false;
initial_poses = 8*(rand(3,numRobots).*[0.5;0.5;0]) + [0.5;0.5;0];
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