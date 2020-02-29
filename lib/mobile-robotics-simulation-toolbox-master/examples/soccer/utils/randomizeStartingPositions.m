%% Script to randomize starting positions for soccer simulation
% Copyright 2019 The MathWorks, Inc.

% Initial ball position
initBallPos = initBallPos + 2*randn(1,2);

% Initial robot player positions
initialPoses = initialPoses + randn(size(initialPoses)).*[3 3 pi/4];