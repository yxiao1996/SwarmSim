% Initialization script for Soccer Simulation example
% Copyright 2019 The MathWorks, Inc.

%% Load bus data types
clear
load soccerBusTypes;

%% Soccer field parameters
% X and Y limits for the field
fieldLimitsX = [0 92];
fieldLimitsY = [0 47];

% Define the field as an Occupancy Map
map = robotics.OccupancyGrid(zeros(fieldLimitsY(2),fieldLimitsX(2)));

% Goal Post Parameters (X, Y, Object Index)
goalPosts = [ 0 27.15 2; 
              0 19.85 2; 
             92 27.15 3; 
             92 19.85 3];
fieldCenter = [mean(fieldLimitsX) mean(fieldLimitsY)];
homeGoalPosition = [fieldLimitsX(1) mean(fieldLimitsY)];
awayGoalPosition = [fieldLimitsX(2) mean(fieldLimitsY)];

%% Simulation Parameters
sampleTime = 0.1;

% Robot definitions and dimensions
numRobots = 6;
robotRadius = 0.8;
wheelRadius = 0.2;
robotColors = [1 0 0;0 0 1;1 0 0;0 0 1;1 0 0;0 0 1];

% Initial Robot poses (X, Y, Theta)
initialPoses = [20 20 0;
                80 30 pi;
                32 29 0;
                62 13 pi; 
                32 18 0;
                62 26 pi];

% Initial Ball Position
initBallPos = fieldCenter;

% Initial Game State
initGameState = struct('possession',0, ...
                       'state',GameState.InPlay, ...
                       'score',[0;0]);
                   
% Ball kicking noise (multiplying factor for 'randn' function)
ballVelNoise = 0.5;
ballAngleNoise = pi/24;

ballThresh = robotRadius + 0.3; % Distance to grab the ball
ballCarryFactor = 0.9;          % Speed penalty when carrying the ball
outOfBoundsDist = 2;            % Distance to place ball back in bounds

% Randomize initial conditions
randomizeStartingPositions;

%% Object Detector and Robot Detector sensor parameters
objDetectorOffset = [0 0];
objDetectorAngle = 0;
objDetectorFOV = pi/3;
objDetectorRange = 100;
objColors = [0 0.75 0; 1 0 0; 0 0 1];
objMarkers = 'o^^';
objDetectorMaxHits = 5;

robotDetectorOffset = [0 0];
robotDetectorAngle = 0;
robotDetectorFOV = pi;
robotDetectorRange = 20;
robotDetectorMaxHits = 5;

%% Behavior Logic Parameters
% General parameters
distThresh = 1;         % Threshold distance to track points [m]
angThresh = pi/16;      % Threshold angle to track rotation [rad]
goalThresh = 20;        % Threshold distance from the goal to kick ball [m]

% Attacker parameters
attackerKickSpeed = 3;              % Kick speed for attacking
attackerMinGoalDist = 5;            % Distance before taking the ball away from goal and try kick again
attackerMaxGoalDist = 10;           % Distance away from goal to travel before kicking again
dribbleTime = 25;                   % Max time before kicking the ball while dribbling
dribbleKickSpeed = 2;               % Kick speed for dribbling

% Defender parameters
defenderHomePoses = [20 28.5 0;
                     20 18.5 0]';   % Defender poses for home team
defenderAwayPoses = [72 28.5 pi;
                     72 18.5 pi]';  % Defender poses for away team
defenderKickSpeed = 5;              % Kick speed for defending

% Goalkeeper parameters
goalkeeperPoses = [ 2 23.5 0;
                   90 23.5 pi]';    % Goalkeeper poses for home and away teams
goalkeeperKickSpeed = 10;           % Kick speed for goalkeeping
