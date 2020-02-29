% Parameters for Navigation Example
% Copyright 2019 The MathWorks, Inc.

%% Main parameters
load exampleMap
robotRadius = 0.25;
sampleTime = 0.1;

%% Lidar sensor parameters
scanAngles = [-3*pi/8;-pi/4;-pi/8;0;pi/8;pi/4;3*pi/8];
maxRange = 6;
lidarNoiseVariance = 0.1^2;
lidarNoiseSeeds = randi(intmax,size(scanAngles));

%% Path planner parameters
prmConnectionDistance = 3;
prmNumNodes = 100;
prmMaxWaypoints = 100;

%% Path follower parameters
followerLinSpeed = 0.3;
followerMaxAngSpeed = 1;
followerLookaheadDistance = 1.5;
goalCheckDistance = 0.2;

%% Reinforcement learning agent parameters
load savedAgent
maxAngSpeed = 0.3;
maxLinSpeed = 0.3;

%% Initial conditions
initX = 2;
initY = 3;
initTheta = 0;