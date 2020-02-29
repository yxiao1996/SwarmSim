%% Lidar Sensor Example
% Copyright 2018-2019 The MathWorks, Inc.
close all

%% Create environment
load exampleMap

% Create lidar sensor
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-pi/2,pi/2,9);
lidar.maxRange = 5;

% Create visualizer
viz = Visualizer2D;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);

%% Simulation parameters
sampleTime = 0.1;              % Sample time [s]
initPose = [1; 3; pi/4];        % Initial pose (x y theta)

% Initialize time, input, and pose arrays
tVec = 0:sampleTime:10;         % Time array
vxRef = 0.3*ones(size(tVec));   % Reference x speed
vyRef = 0.15*ones(size(tVec));  % Reference y speed
wRef = zeros(size(tVec));       % Reference angular speed
wRef(tVec < 5) = -0.75;
wRef(tVec >=5) = 0.75;
ref = [vxRef;vyRef;wRef];
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

%% Simulation loop
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec)   
    % Convert the reference speeds to world coordinates
    vel = bodyToWorld(ref(:,idx-1),pose(:,idx-1));
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime;
    
    % Update lidar and visualization
    ranges = lidar(pose(:,idx));
    viz(pose(:,idx),ranges)
    waitfor(r);
end