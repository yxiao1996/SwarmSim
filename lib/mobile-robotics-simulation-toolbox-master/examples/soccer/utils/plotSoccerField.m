% Script for plotting the Soccer field over the Robot Visualizer
%
% Copyright 2019 The MathWorks, Inc.

% Find the figure number to start plotting over
obj = findobj('Tag','MultiRobotEnvironment');
figNum = obj.Number;
figure(figNum);
hold on

% Title
title('Robot Soccer Simulation');

% Center Line
xline(46,'k-','LineWidth',2);

% Center circle
viscircles([46 23.5],9.15,'Color','k');

% Draw outer boundary
rectangle('Position', [0 0 92 47],'LineWidth', 2);

% Remove default labels and ticks
xlabel('');
ylabel('');
yticks('');
xticks('');

% Crop to field dimensions
axis equal
xlim([0 92]);
ylim([0 47]);

hold off