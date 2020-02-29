%% EXAMPLE: Four-wheel steering continuous simulation
% Copyright 2018 The MathWorks, Inc.

%% Define vehicle
wheelRadius = 0.1;          % Wheel radius [m]
wheelDists = [0.3, 0.25];   % Front and rear wheel distances to CG [m]
vehicle = FourWheelSteering(wheelRadius,wheelDists);

%% Run a continuous simulation using ODE45
initPose = [0; 0; pi/4];    % Initial pose (x y theta)
ikMode = 1;     % Inverse kinematics mode
                % 1: Front steering
                % 2: Zero sideslip
                % 3: Parallel steering;
tspan = [0 10];
[t,pose] = ode45(@(t,y)fourWheelSteerDynamics(t,y,vehicle,ikMode), ...
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
function dy = fourWheelSteerDynamics(t,y,vehicle,ikMode)   
    
    % Set desired velocities
    vxRef = 0.2;
    vyRef = 0.1;
    if t < 5
       wRef = -0.5; 
    else
       wRef = 0.5;
    end
    
    % Solve inverse kinematics to find wheel speeds and steer angles
    if ikMode == 1
        [wheelSpeeds,steerAngles] = ...
            inverseKinematicsFrontSteer(vehicle,vxRef,wRef);
    elseif ikMode == 2
        [wheelSpeeds,steerAngles] = ...
            inverseKinematicsZeroSideslip(vehicle,vxRef,wRef);
    else
        [wheelSpeeds,steerAngles] = ...
            inverseKinematicsParallelSteer(vehicle,vxRef,vyRef); 
    end
    
    % Calculate forward kinematics in world coordinates
    velB = forwardKinematics(vehicle,wheelSpeeds,steerAngles);
    dy = bodyToWorld(velB,y);
end