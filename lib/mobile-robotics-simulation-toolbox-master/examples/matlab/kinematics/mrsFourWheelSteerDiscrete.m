%% EXAMPLE: Four-wheel steering discrete simulation
% Copyright 2018 The MathWorks, Inc.

%% Define vehicle
wheelRadius = 0.1;          % Wheel radius [m]
wheelDists = [0.3, 0.25];   % Front and rear wheel distances to CG [m]
vehicle = FourWheelSteering(wheelRadius,wheelDists);

%% Simulation parameters
sampleTime = 0.01;       % Sample time [s]
initPose = [0; 0; pi/4]; % Initial pose (x y theta)
ikMode = 1;     % Inverse kinematics mode:
                % 1: Front steering
                % 2: Zero sideslip
                % 3: Parallel steering

% Initialize time, input, and pose arrays
tVec = 0:sampleTime:10;         % Time array
vxRef = 0.2*ones(size(tVec));   % Reference x speed
vyRef = 0.1*ones(size(tVec));   % Reference y speed
wRef = zeros(size(tVec));       % Reference angular speed
wRef(tVec < 5) = -0.5;
wRef(tVec >=5) = 0.5;
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

%% Simulation loop
for idx = 2:numel(tVec) 
    % Solve inverse kinematics to find wheel speeds and steer angles
    if ikMode == 1
        [wheelSpeeds,steerAngles] = ...
            inverseKinematicsFrontSteer(vehicle,vxRef(idx-1),wRef(idx-1));
    elseif ikMode == 2
        [wheelSpeeds,steerAngles] = ...
            inverseKinematicsZeroSideslip(vehicle,vxRef(idx-1),wRef(idx-1));
    else
        [wheelSpeeds,steerAngles] = ...
            inverseKinematicsParallelSteer(vehicle,vxRef(idx-1),vyRef(idx-1)); 
    end
    
    % Compute the velocities
    velB = forwardKinematics(vehicle,wheelSpeeds,steerAngles);
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