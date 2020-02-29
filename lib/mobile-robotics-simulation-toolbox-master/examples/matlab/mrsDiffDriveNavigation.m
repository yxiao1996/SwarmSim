%% EXAMPLE: Differential Drive Path Planning and Navigation
% In this example, a path is found in an occupancy grid using a 
% probabilistic roadmap (PRM) and followed using Pure Pursuit
% 
% Copyright 2018-2019 The MathWorks, Inc.

%% Define Vehicle
R = 0.1;                % Wheel radius [m]
L = 0.5;                % Wheelbase [m]
dd = DifferentialDrive(R,L);

%% Simulation parameters
sampleTime = 0.1;               % Sample time [s]
tVec = 0:sampleTime:25;         % Time array

initPose = [2;2;0];             % Initial pose (x y theta)
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

%% Path planning
% Load map and inflate it by a safety distance
close all
load exampleMap
inflate(map,R);

% Create a Probabilistic Road Map (PRM)
planner = mobileRobotPRM(map);
planner.NumNodes = 75;
planner.ConnectionDistance = 5;

% Find a path from the start point to a specified goal point
startPoint = initPose(1:2)';
goalPoint  = [11, 11];
waypoints = findpath(planner,startPoint,goalPoint);
show(planner)

%% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.35;
controller.DesiredLinearVelocity = 0.75;
controller.MaxAngularVelocity = 1.5;

%% Create visualizer 
load exampleMap % Reload original (uninflated) map for visualization
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'map';

%% Simulation loop
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = controller(pose(:,idx-1));
    [wL,wR] = inverseKinematics(dd,vRef,wRef);
    
    % Compute the velocities
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx),waypoints)
    waitfor(r);
end