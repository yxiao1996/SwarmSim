function maxNumScans = getMaxNumScans()
% Search for the max size of all laser range sensors in a model
%
% Copyright 2019 The MathWorks, Inc.

vizBlocks = find_system(bdroot,'FollowLinks','on','LookUnderMasks','on','MaskType','Multi-Robot Visualizer');
numBlocks = numel(vizBlocks);
numScans = zeros(numBlocks,1);
for idx = 1:numBlocks
   numScans(idx) = numel(evalin('base',get_param(vizBlocks{idx},'scanAngles'))); 
end
maxNumScans = max(numScans);

end
