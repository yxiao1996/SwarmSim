%% Multi-Robot Sensor Example
% Copyright 2018-2019 The MathWorks, Inc.

% Create a multi-robot environment
numRobots = 3;
env = MultiRobotEnv(numRobots);
env.robotRadius = [0.2,0.5,0];
env.showTrajectory = [true;true;true];
env.hasWaypoints = true;
load exampleMap
env.mapName = 'map';

%% Create sensors
sensor1 = MultiRobotLidarSensor;
sensor1.robotIdx = 1;
sensor1.sensorOffset = [0,0];
sensor1.scanAngles = linspace(-pi/2,pi/2,9);
sensor1.maxRange = 3;
attachLidarSensor(env,sensor1);

sensor2 = ObjectDetector;
sensor2.fieldOfView = pi/4;
attachObjectDetector(env,2,sensor2);

sensor3 = MultiRobotLidarSensor;
sensor3.robotIdx = 3;
sensor3.sensorOffset = [0,0];
sensor3.scanAngles = linspace(-pi/4,pi/4,10);
sensor3.maxRange = 10;
attachLidarSensor(env,sensor3);

%% Define waypoints, objects, and initial poses
waypoints = [2 4;
             12 12;
             1 11];
objects = [2, 8, 1;
           8, 11, 2;
           8, 4, 3];
env.objectColors = [1 0 0;0 1 0;0 0 1];
env.objectMarkers = 'so^';
pose1 = [2;5;pi/2];
pose2 = [2;8;-pi];
pose3 = [10;10;0];
env.Poses = [pose1 pose2 pose3];

%% Now loop through the animation
for idx = 1:100
    % Step the sensors
    ranges1     = sensor1();
    detections2 = sensor2(pose2,objects);
    ranges3     = sensor3();
    
    % Step the visualizer
    % In multiple commands
    % env(1,pose1, waypoints, ranges1, objects);
    % env(2,pose2, waypoints, [], objects);
    % env(3,pose3, waypoints, ranges3,  objects);
    % In single command
    env([1,2,3],[pose1,pose2,pose3],waypoints,{ranges1,[],ranges3},objects)
    
    % Update poses
    pose1 = pose1 + [0.075;-0.025;-pi/32];
    pose2 = pose2 + [0.025;0.005;pi/64];
    pose3 = pose3 + [0;0;pi/24];
    
end