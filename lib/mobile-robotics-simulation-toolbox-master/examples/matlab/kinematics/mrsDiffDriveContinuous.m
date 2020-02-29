%% EXAMPLE: Differential Drive continuous simulation
% Copyright 2018 The MathWorks, Inc.

%% Define vehicle
R = 0.1;                % Wheel radius [m]
L = 0.5;                % Wheelbase [m]
dd = DifferentialDrive(R,L);

%% Run a continuous simulation using ODE45
initPose = [0 0 pi/4];  % Initial pose (x y theta)
tspan = [0 10];
[t,pose] = ode45(@(t,y)diffDriveDynamics(t,y,dd),tspan,initPose);
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
function dy = diffDriveDynamics(t,y,vehicle)
    
    % Set desired velocities and solve inverse kinematics
    vDes = 0.2;
    if t < 5
       wDes = -0.5; 
    else
       wDes = 0.5;
    end
    [wL,wR] = vehicle.inverseKinematics(vDes,wDes);
    
    % Calculate forward kinematics and convert the speeds to global
    % coordinates
    [v,w] = vehicle.forwardKinematics(wL,wR);
    velB = [v;0;w];            % Body velocities [vx;vy;w]
    dy = bodyToWorld(velB,y);  % Convert from body to world
    
end


