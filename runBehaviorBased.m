%% a test script for simulator class
clear all;
close all;
% generate map for the simulation
size = 15;
resolution = 10;
numObstacles = 8;
space = 5;
%p = zeros(size*resolution);
map_gen = MapGenerate(size,size,space,resolution);
[p,map_gen] = map_gen.addBounds(2);
for i = 1:numObstacles
    %p = add_random_circle(p);
    [p,map_gen] = map_gen.addRandomObstacle(1.0,0.5);
end
map = binaryOccupancyMap(p,resolution);

%% specify some parameters
numRobots = 5;
numSensors = 25;
sensorRange = 2;
showTraj = false;
initial_poses = 8*(rand(3,numRobots).*[0.5;0.5;0]) + [0.5;0.5;0];
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
%% virtual structure simulation
sim = BehaviorBasedSimulation(map,swarmInfo);
for i = 1:500
    sim = sim.step();
    axis([0 15 0 15])
    pause(0.02);
end

function p = add_random_circle(p)
    world_size = size(p,1);
    circle_size = floor(world_size/10);
    x = rand*(2*circle_size) - 1*circle_size + world_size/2;
    y = rand*(2*circle_size) - 1*circle_size + world_size/2;
    xy = [x y];
    for i = 1:world_size
        for j = 1:world_size
            dist = sqrt((i-x)^2 + (y-j)^2);
            if (dist < circle_size)
                p(i,j) = 1;
            end
        end
    end
end