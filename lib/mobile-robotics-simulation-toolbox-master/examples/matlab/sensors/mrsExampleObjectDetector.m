%% Object Detector Example
% Copyright 2018-2019 The MathWorks, Inc.
clc
close all

%% Create environment
load exampleMap

% Create objects [x,y,label]
objects = [2, 8, 1; ...
           8, 11, 2; ...
           8, 4, 3];

% Create object Detector sensor
detector = ObjectDetector;
detector.fieldOfView = pi/4;

% Create visualizer
viz = Visualizer2D;
viz.mapName = 'map';
attachObjectDetector(viz,detector);
viz.objectColors = [1 0 0;0 1 0;0 0 1];
viz.objectMarkers = 'so^';

%% Simulation parameters
sampleTime = 0.05;             % Sample time [s]
initPose = [6.5; 8; 0];        % Initial pose (x y theta)

% Initialize time, input, and pose arrays
tVec = 0:sampleTime:10;         % Time array
vxRef = zeros(size(tVec));      % Reference x speed
vyRef = zeros(size(tVec));  % Reference y speed
wRef = 0.75*ones(size(tVec));   % Reference angular speed
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
    
    % Update object detector and visualization
    detections = detector(pose(:,idx),objects);
    viz(pose(:,idx),objects)
    
    % Display object detections every 10th iteration
    if mod(idx,10) == 0
        if ~isempty(detections)
            nearestLabel = detections(1,3);
            disp(['Nearest object is of label ' num2str(nearestLabel)]); 
        else
            disp('No objects detected'); 
        end
    end  
    
    waitfor(r);
end