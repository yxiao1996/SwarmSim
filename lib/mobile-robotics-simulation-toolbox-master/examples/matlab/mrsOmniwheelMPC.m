%% EXAMPLE: Vehicle following waypoints using nonlinear Model Predictive Control (MPC)
%
% Copyright 2018-2019 The MathWorks, Inc.

%% Define Vehicle
wheelRadius = 0.1;                      % Wheel radius [m]
robotRadius = 0.25;                     % Robot radius [m]
wheelAngles = [0, 2*pi/3, -2*pi/3];     % Wheel angles [rad]
vehicle = TripleOmniwheel(wheelRadius,robotRadius,wheelAngles);

%% Simulation parameters
sampleTime = 0.1;               % Sample time [s]
tVec = 0:sampleTime:15;         % Time array
initPose = [0;0;0];             % Initial pose (x y theta)
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

% Define waypoints, times, and trajectory
times = [0; 3; 6; 9; 12; 15];
waypoints = [0,0,0; 2,2,pi/4; 4,2,pi/2; 
             2,4,pi/2; 0.5,3,-pi/2; 0.5,3,-pi/2]; % [x y theta]
ref = interp1(times,waypoints,tVec,'linear'); % Try other interpolation methods!

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;

%% Create the nonlinear MPC Controller
nx = 3; % Number of states
ny = 3; % Number of outputs
nu = 3; % Number of inputs
controller = nlmpc(nx,ny,nu);

p = 5;  % Prediction horizon
m = 3;  % Control horizon
controller.Ts = sampleTime;
controller.PredictionHorizon = p;
controller.ControlHorizon = m;

% Add dynamic model for nonlinear MPC
controller.Model.StateFcn = @(xk,u)discreteDynamics(xk,u,sampleTime); 
controller.Model.IsContinuousTime = false;

% Add weights
controller.Weights.ManipulatedVariables     = [1, 1, 1];
controller.Weights.ManipulatedVariablesRate = [10, 10, 10];
controller.Weights.OutputVariables          = [100, 100, 50];

% Add constraints
% X velocity
controller.MV(1).Max =  0.9;     controller.MV(1).RateMax =  0.2;
controller.MV(1).Min = -0.9;     controller.MV(1).RateMin = -0.2; 
% Y velocity
controller.MV(2).Max =  0.9;     controller.MV(2).RateMax =  0.2;
controller.MV(2).Min = -0.9;     controller.MV(2).RateMin = -0.2; 
% Z velocity
controller.MV(3).Max =  pi/2;    controller.MV(3).RateMax =  pi/4;
controller.MV(3).Min = -pi/2;    controller.MV(3).RateMin = -pi/4;

%% Simulation loop
close all
r = rateControl(1/sampleTime);
u = zeros(3,numel(tVec));
wheelSpeeds = zeros(3,numel(tVec));

ref(end+1:end+p,:) = repmat(ref(end,:),[p 1]); % Extend reference by prediction horizon

for idx = 2:numel(tVec)  
    % Run the MPC controller
    [u(:,idx),~,mpcinfo] = nlmpcmove(controller,pose(:,idx-1), ...
                                     u(:,idx-1),ref(idx:idx+p-1,:));
    
    % Convert velocities to wheel speeds
    wheelSpeeds(:,idx) = inverseKinematics(vehicle,u(:,idx));
    
    % Compute the velocities
    velB = forwardKinematics(vehicle,wheelSpeeds(:,idx));
    vel = bodyToWorld(velB,pose(:,idx-1));
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    
    % Update visualization, including MPC prediction overlays
    viz(pose(:,idx),waypoints)
    hold on
    if exist('hmpc','var')
        delete(hmpc)
    end
    hmpc = plot(mpcinfo.Yopt(2:end,1),mpcinfo.Yopt(2:end,2),'bs');
    hold off
    waitfor(r);
end

%% Control plots
figure
subplot(311), plot(tVec, u(1,:)), ylabel('X Vel [m/s]');
title('World Velocity Trajectory');
subplot(312), plot(tVec, u(2,:)), ylabel('Y Vel[m/s]');
subplot(313), plot(tVec, u(3,:)), ylabel('Angular Vel [rad/s]');
xlabel('Time [s]');
figure
plot(tVec, wheelSpeeds)
title('Wheel Speed Trajectory')
xlabel('Time [s]');
ylabel('Wheel Speeds [rad/s]');
legend('Wheel 1','Wheel 2','Wheel 3');


%% Helper functions 
function xk1 = discreteDynamics(xk,u,dt)

    % x[k+1] = x[k] + B*u[k]*dt
    % Where: x[k] = [x;y;theta], u[k] = [vx;vy;w]
    theta = xk(3);
    M = [cos(theta), -sin(theta), 0;
         sin(theta),  cos(theta), 0;
             0     ,      0,      1];
    xk1 = xk + M*u*dt;
end