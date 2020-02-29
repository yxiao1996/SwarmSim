%% EXAMPLE: Four-wheel vehicle with zero sideslip steering 
% (equal and opposite front and rear steering), following waypoints 
% using the Pure Pursuit algorithm
%
% Copyright 2018-2019 The MathWorks, Inc.

%% Define Vehicle
wheelRadius = 0.1;             % Wheel radius [m]
frontLen = 0.25;               % Distance from CG to front wheels [m]
rearLen = 0.2;                 % Distance from CG to rear wheels [m]
vehicle = FourWheelSteering(wheelRadius,[frontLen rearLen]);

%% Simulation parameters
sampleTime = 0.1;               % Sample time [s]
tVec = 0:sampleTime:15;         % Time array

initPose = [0;0;0];             % Initial pose (x y theta)
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

% Define waypoints
waypoints = [0,0; 2,2; 4,2; 2,4; 0.5,3];

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;

% Create arrays
wArray = zeros(2,numel(tVec)-1);      % Wheel speeds
phiArray = zeros(2,numel(tVec)-1);    % Steer angles

%% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.3;
controller.DesiredLinearVelocity = 0.75;
controller.MaxAngularVelocity = 0.75;

%% Simulation loop
close all
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = controller(pose(:,idx-1));
    [wheelSpeeds,steerAngles] = ...
                 inverseKinematicsZeroSideslip(vehicle,vRef,wRef);
                 %inverseKinematicsFrontSteer(vehicle,vRef,wRef);
    
    % Compute the velocities
    velB = forwardKinematics(vehicle,wheelSpeeds,steerAngles);
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx),waypoints)
    
    % Update control input arrays
    wArray(:,idx-1) = wheelSpeeds;
    phiArray(:,idx-1) = steerAngles;
    waitfor(r);
end

%% Visualize the control inputs
figure
subplot(2,1,1)
plot(tVec(1:end-1),wArray(1,:)*30/pi, ...
     tVec(1:end-1),wArray(2,:)*30/pi);
title('Control Inputs')
xlabel('Time [s]'), ylabel('Wheel Speeds [rpm]')
legend('Front','Rear')
subplot(2,1,2)
plot(tVec(1:end-1),phiArray(1,:)*180/pi, ...
     tVec(1:end-1),phiArray(2,:)*180/pi);
xlabel('Time [s]'), ylabel('Steer Angle [deg]')
legend('Front','Rear')