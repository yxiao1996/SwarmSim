%% EXAMPLE: Generic Omniwheel discrete simulation
% Copyright 2018 The MathWorks, Inc.

%% Define Vehicle
wheelRadius = 0.1;                      % Wheel radius [m]
wheelPositions = [1 1;-1 1;-1 -1;1 -1]; % Wheel (X,Y) positions [m]
wheelAngles = [0; pi/4; -pi/4; 0];      % Wheel angles [rad]
vehicle = GenericOmniwheel(wheelRadius,wheelPositions,wheelAngles);

%% Simulation parameters
sampleTime = 0.01;       % Sample time [s]
initPose = [0; 0; pi/4]; % Initial pose (x y theta)
bodyMode = true;         % True for body frame speeds, false for world frame speeds

% Initialize time, input, and pose arrays
tVec = 0:sampleTime:10;         % Time array
vxRef = 0.2*ones(size(tVec));   % Reference x speed
vyRef = 0.1*ones(size(tVec));   % Reference y speed
wRef = zeros(size(tVec));       % Reference angular speed
wRef(tVec < 5) = -0.5;
wRef(tVec >=5) = 0.5;
ref = [vxRef;vyRef;wRef];
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

%% Simulation loop
for idx = 2:numel(tVec) 
    % Solve inverse kinematics to find wheel speeds
    if bodyMode
        wheelSpeeds = inverseKinematics(vehicle,ref(:,idx-1));
    else
        refWorld = worldToBody(ref(:,idx-1),pose(:,idx-1));
        wheelSpeeds = inverseKinematics(vehicle,refWorld);
    end
    
    % Compute the velocities
    velB = forwardKinematics(vehicle,wheelSpeeds);
    vel = bodyToWorld(velB,pose(:,idx-1));
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime;
end

%% Display results
close all
figure
hold on
plot(pose(1,1),pose(2,1),'ro', ...
     pose(1,end),pose(2,end),'go', ...
     pose(1,:),pose(2,:),'b-');
axis equal
title('Vehicle Trajectory');
xlabel('X [m]')
ylabel('Y [m]')
legend('Start','End','Trajectory')