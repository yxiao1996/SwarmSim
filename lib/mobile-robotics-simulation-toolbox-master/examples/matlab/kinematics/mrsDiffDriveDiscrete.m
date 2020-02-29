%% EXAMPLE: Differential Drive discrete simulation
% Copyright 2018 The MathWorks, Inc.

%% Define Robot
R = 0.1;                % Wheel radius [m]
L = 0.5;                % Wheelbase [m]
dd = DifferentialDrive(R,L);

%% Simulation parameters
sampleTime = 0.01;              % Sample time [s]
initPose = [0;0;pi/4];          % Initial pose (x y theta)

% Initialize time, input, and pose arrays
tVec = 0:sampleTime:10;         % Time array
vRef = 0.2*ones(size(tVec));    % Reference linear speed
wRef = zeros(size(tVec));       % Reference angular speed
wRef(tVec < 5) = -0.5;
wRef(tVec >=5) = 0.5;
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

%% Simulation loop
for idx = 2:numel(tVec) 
    % Solve inverse kinematics to find wheel speeds
    [wL,wR] = inverseKinematics(dd,vRef(idx-1),wRef(idx-1));
    
    % Compute the velocities
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    
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