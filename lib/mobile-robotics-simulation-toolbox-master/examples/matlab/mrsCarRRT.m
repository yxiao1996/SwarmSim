%% EXAMPLE: Path planning and following for a car-like vehicle using
% the Rapidly-exploring Random Tree (RRT) algorithm
%
% Copyright 2019 The MathWorks, Inc.

%% Define Vehicle
wheelRadius = 0.05;     % Wheel radius [m]
frontLen = 0.25;        % Distance from CG to front wheels [m]
rearLen = 0.25;         % Distance from CG to rear wheels [m]
vehicle = FourWheelSteering(wheelRadius,[frontLen rearLen]);

%% Simulation parameters
sampleTime = 0.1;               % Sample time [s]
tVec = 0:sampleTime:100;        % Time array

startPose = [2 2 0];            % Start pose [x y theta]
goalPose = [18 17 -pi/2];       % Goal pose [x y theta]

% Create visualizer
viz = Visualizer2D;
viz.robotRadius = frontLen;
viz.hasWaypoints = false;
viz.mapName = 'map';

%% Create RRT Planner
load complexMap
inflate(map,0.25); % Inflate the map for planning

% State space
ss = stateSpaceDubins;
ss.MinTurningRadius = 0.75;
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

% State validator
sv = validatorOccupancyMap(ss);
sv.Map = map;
sv.ValidationDistance = 0.1;

% Path planner
planner = plannerRRT(ss,sv);
planner.MaxConnectionDistance = 2.5;

%% Plan a path and visualize it
[plannedPath,solInfo] = plan(planner,startPose,goalPose);
if plannedPath.NumStates < 1
    disp('No path found. Please rerun the example');
end
interpolate(plannedPath,round(2*plannedPath.pathLength)); % Interpolate to approx. 2 waypoints per meter

load complexMap % Reload map without inflation for display
viz(startPose)
visualizeRRTPath(plannedPath,solInfo,ss); % Helper function to visualize

%% Define Pure Pursuit controller for path following
waypoints = plannedPath.States(:,1:2);
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.25;
controller.DesiredLinearVelocity = 1;
controller.MaxAngularVelocity = 3;

%% Path following simulation loop
r = rateControl(5/sampleTime); % Run at 5x speed
pose = zeros(3,numel(tVec));
pose(:,1) = startPose;
idx = 2;
dist = Inf;
while (idx < numel(tVec)) && (dist > 0.35)
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = controller(pose(:,idx-1));
    [wheelSpeeds,steerAngles] = inverseKinematicsFrontSteer(vehicle,vRef,wRef);
    wheelSpeeds = wheelSpeeds([1 1]); % Use front wheel speed for both
    
    % Compute the velocities
    velB = forwardKinematics(vehicle,wheelSpeeds,steerAngles);
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime;
    
    % Update visualization
    viz(pose(:,idx))
    
    % Calculate distance to goal and update loop
    dist = norm( pose(1:2,idx) - goalPose(1:2)' );
    idx = idx+1;
    waitfor(r);
end

%% Helper function to visualize path
function visualizeRRTPath(plannedPath,solInfo,ss)
hold on
% Plot the path from start to goal
plot(plannedPath.States(:,1),plannedPath.States(:,2),'r--','LineWidth',1.5);
% Interpolate each path segment to be smoother and plot it
tData = solInfo.TreeData;
for idx = 3:3:size(tData,1)-2
    p = navPath(ss,tData(idx:idx+1,:));
    interpolate(p,10);
    plot(p.States(:,1),p.States(:,2),':','Color',[0 0.6 0.9]);
end
hold off
end