%% EXAMPLE: Generic Omniwheel continuous simulation
% Copyright 2018 The MathWorks, Inc.

%% Define Vehicle
wheelRadius = 0.1;                      % Wheel radius [m]
wheelPositions = [1 1;-1 1;-1 -1;1 -1]; % Wheel (X,Y) positions [m]
wheelAngles = [0; pi/4; -pi/4; 0];      % Wheel angles [rad]
vehicle = GenericOmniwheel(wheelRadius,wheelPositions,wheelAngles);

%% Run a continuous simulation using ODE45
initPose = [0; 0; pi/4]; % Initial pose (x y theta)
bodyMode = true;         % True for body frame speeds, false for world frame speeds
tspan = [0 10];
[t,pose] = ode45(@(t,y)genOmniDynamics(t,y,vehicle,bodyMode), ...
                  tspan,initPose);
pose = pose';

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


%% Continuous dynamics
function dy = genOmniDynamics(t,y,vehicle,bodyMode)   
    
    % Set desired velocities
    vxRef = 0.2;
    vyRef = 0.1;
    if t < 5
       wRef = -0.5; 
    else
       wRef = 0.5;
    end
    
    % Solve inverse kinematics
    if bodyMode
        wheelSpeeds = inverseKinematics(vehicle,[vxRef;vyRef;wRef]);
    else
        wheelSpeeds = inverseKinematicsWorld(vehicle,[vxRef;vyRef;wRef],y);
    end
    
    % Calculate forward kinematics in world coordinates
    velB = forwardKinematics(vehicle,wheelSpeeds);
    dy = bodyToWorld(velB,y);
end