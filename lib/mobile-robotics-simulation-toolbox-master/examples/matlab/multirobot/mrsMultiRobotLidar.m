%% Multi-Robot Lidar Example
% Copyright 2018-2019 The MathWorks, Inc.

%% Create a multi-robot environment
numRobots = 10;
env = MultiRobotEnv(numRobots);
env.showTrajectory = false;
env.robotRadius = 0.25;
load exampleMap
env.mapName = 'map';

%% Create a lidar sensor
lidar = MultiRobotLidarSensor;
lidar.robotIdx = 3;
lidar.scanAngles = linspace(-pi,pi,25);
lidar.maxRange = 4;
attachLidarSensor(env,lidar);

%% Animate and show the detections [range, angle, index]
poses = 4*(rand(3,numRobots).*[1;1;pi] - [0.5;0.5;0]) + [9;9;0];
env.Poses = poses;
dTheta = pi/64;
ranges = cell(1,numRobots);
for idx = 1:100
    % Get the current time step's ranges
    scans = lidar();
    ranges{robotIdx} = scans;
    
    % Update the environment and poses
    env(1:numRobots, poses, ranges)
    poses(3,robotIdx) = poses(3,robotIdx) + dTheta;    
end