% Startup script that sets up the necessary paths and opens the 
% Mobile Robotics Simulation Library.
% This script is only needed if this was not installed as a toolbox.
%
% Copyright 2018-2019 The MathWorks, Inc.

% Set up path
addpath(genpath('examples'))
addpath(genpath('src'))
addpath(genpath('doc'))

% Open the getting started guide
edit GettingStarted