%% Initialization script for nonlinear MPC example
%
% Copyright 2018 The MathWorks, Inc.

%% Sample time
sampleTime = 0.1;

%% Vehicle Parameters
wheelRadius = 0.1;
robotRadius = 0.25;
wheelAngles = [0, 2*pi/3, -2*pi/3];

%% Define waypoints, times, and trajectory
times = [0; 3; 6; 9; 12; 15];
waypoints = [0,0,0; 2,2,pi/4; 4,2,pi/2; 
             2,4,pi/2; 0.5,3,-pi/2; 0.5,3,-pi/2]; % [x y theta]

%% Create the nonlinear MPC Controller
nx = 3; % Number of states
ny = 3; % Number of outputs
nu = 3; % Number of inputs
controller = nlmpc(nx,ny,nu);

p = 5; % Prediction horizon
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