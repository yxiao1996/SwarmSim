% Mobile Robot Obstacle Avoidance using Reinforcement Learning
% Agent Training Script
%
% Copyright 2019 The MathWorks, Inc.
 
%% Initialization
clear; close all force; bdclose all; clc;

mdl = 'rlObstacleAvoidance';
mapName = 'exampleMap';
doVisualization = false;
doTraining = true;

open_system(mdl)
rlMobileRobotParams;
load(mapName)

%% Create environment interfaces
Tfinal = 30;
agentBlk = [mdl '/RL Agent'];
obsInfo = rlNumericSpec([numel(scanAngles) 1],...
    'LowerLimit',zeros(numel(scanAngles),1),...
    'UpperLimit',ones(numel(scanAngles),1)*maxRange);
numObservations = obsInfo.Dimension(1);

numActions = 2;
actInfo = rlNumericSpec([numActions 1],...
    'LowerLimit',-1,...
    'UpperLimit',1);

% Build the environment interface object.
env = rlSimulinkEnv(mdl,agentBlk,obsInfo,actInfo);
env.ResetFcn = @(in)rlMobileRobotResetFcn(in,scanAngles,maxRange,mapName);
if doVisualization
    set_param([mdl '/Robot Visualizer'],'Commented','off');
    env.UseFastRestart = 'off';
else
    set_param([mdl '/Robot Visualizer'],'Commented','on');
    env.UseFastRestart = 'on';
end

%% Create DDPG agent
statePath = [
    imageInputLayer([numObservations 1 1], 'Normalization', 'none', 'Name', 'State')
    fullyConnectedLayer(50, 'Name', 'CriticStateFC1')
    reluLayer('Name', 'CriticRelu1')
    fullyConnectedLayer(25, 'Name', 'CriticStateFC2')];
actionPath = [
    imageInputLayer([numActions 1 1], 'Normalization', 'none', 'Name', 'Action')
    fullyConnectedLayer(25, 'Name', 'CriticActionFC1')];
commonPath = [
    additionLayer(2,'Name', 'add')
    reluLayer('Name','CriticCommonRelu')
    fullyConnectedLayer(1, 'Name', 'CriticOutput')];

criticNetwork = layerGraph();
criticNetwork = addLayers(criticNetwork,statePath);
criticNetwork = addLayers(criticNetwork,actionPath);
criticNetwork = addLayers(criticNetwork,commonPath);
criticNetwork = connectLayers(criticNetwork,'CriticStateFC2','add/in1');
criticNetwork = connectLayers(criticNetwork,'CriticActionFC1','add/in2');

% Create the critic representation
criticOpts = rlRepresentationOptions('LearnRate',1e-3,'L2RegularizationFactor',1e-4,'GradientThreshold',1);
critic = rlRepresentation(criticNetwork,obsInfo,actInfo,'Observation',{'State'},'Action',{'Action'},criticOpts);

%% Construct the actor
actorNetwork = [
    imageInputLayer([numObservations 1 1], 'Normalization', 'none', 'Name', 'State')
    fullyConnectedLayer(50, 'Name', 'actorFC1')
    reluLayer('Name','actorReLU1')
    fullyConnectedLayer(50, 'Name', 'actorFC2')
    reluLayer('Name','actorReLU2')
    fullyConnectedLayer(2, 'Name', 'actorFC3')
    tanhLayer('Name', 'Action')];

% Create the actor representation
actorOptions = rlRepresentationOptions('LearnRate',1e-4,'L2RegularizationFactor',1e-4,'GradientThreshold',1);
actor = rlRepresentation(actorNetwork,obsInfo,actInfo,'Observation',{'State'},'Action',{'Action'},actorOptions);

%% Create the DDPG agent
agentOpts = rlDDPGAgentOptions(...
    'SampleTime',sampleTime,...
    'TargetSmoothFactor',1e-3,...
    'DiscountFactor',0.995, ...
    'MiniBatchSize',128, ...
    'ExperienceBufferLength',1e6); 
agentOpts.NoiseOptions.Variance = 0.1;
agentOpts.NoiseOptions.VarianceDecayRate = 1e-5;
agent = rlDDPGAgent(actor,critic,agentOpts);

%% Train Agent
maxEpisodes = 10000;
maxSteps = ceil(Tfinal/sampleTime);
trainOpts = rlTrainingOptions(...
    'MaxEpisodes',maxEpisodes, ...
    'MaxStepsPerEpisode',maxSteps, ...
    'ScoreAveragingWindowLength',50, ...
    'StopTrainingCriteria','AverageReward', ...
    'StopTrainingValue',80, ...
    'Verbose', true, ...
    'Plots','training-progress');

if doTraining
    % Train the agent.
    trainingStats = train(agent,env,trainOpts);
else
    % Load pretrained agent for the example.
    load('savedAgent.mat','agent');
end

%% Validate Trained Agent
set_param([mdl '/Robot Visualizer'],'Commented','off');
rlSimulationOptions('MaxSteps',maxSteps,'StopOnError','on');
experiences = sim(env,agent);