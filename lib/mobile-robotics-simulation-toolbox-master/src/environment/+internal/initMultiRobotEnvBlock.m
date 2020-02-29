% Configure visual tags
visualSub = [gcb '/Visuals'];
radiusBlocks = find_system(visualSub,'FollowLinks','on','LookUnderMasks','on','BlockType','From');
for idx = 1:numel(radiusBlocks)
    lh = get_param(radiusBlocks{idx},'LineHandles');
    delete_line(lh.Outport);
end
delete_block(radiusBlocks);
vecBlock = [visualSub '/Vector Concatenate'];
set_param(vecBlock,'NumInputs',num2str(numRobots));
for idx = 1:numRobots
    blkName = [visualSub '/From' num2str(idx)];
    blkPos = [0 30 200 50] + (idx-1)*[0 30 0 30];
    add_block('simulink/Signal Routing/From',blkName);
    set_param(blkName,'GotoTag',['slMultiRobotEnv_Visuals_' num2str(idx)]);
    set_param(blkName,'Position',blkPos);
    add_line(visualSub,['From' num2str(idx) '/1'],['Vector Concatenate/' num2str(idx)]);
end

% Configure pose tags
poseSub = [gcb '/Poses'];
poseBlocks = find_system(poseSub,'FollowLinks','on','LookUnderMasks','on','BlockType','From');
for idx = 1:numel(poseBlocks)
    lh = get_param(poseBlocks{idx},'LineHandles');
    delete_line(lh.Outport);
end
delete_block(poseBlocks);
vecBlock = [poseSub '/Vector Concatenate'];
set_param(vecBlock,'NumInputs',num2str(numRobots));
for idx = 1:numRobots
    blkName = [poseSub '/From' num2str(idx)];
    blkPos = [0 30 200 50] + (idx-1)*[0 30 0 30];
    add_block('simulink/Signal Routing/From',blkName);
    set_param(blkName,'GotoTag',['slMultiRobotEnv_Pose_' num2str(idx)]);
    set_param(blkName,'Position',blkPos);
    add_line(poseSub,['From' num2str(idx) '/1'],['Vector Concatenate/' num2str(idx)]);
end

% Configure range tags
rangeSub = [gcb '/Ranges'];
rangeBlocks = find_system(rangeSub,'FollowLinks','on','LookUnderMasks','on','BlockType','From');
for idx = 1:numel(rangeBlocks)
    lh = get_param(rangeBlocks{idx},'LineHandles');
    delete_line(lh.Outport);
end
delete_block(rangeBlocks);
vecBlock = [rangeSub '/Vector Concatenate'];
set_param(vecBlock,'NumInputs',num2str(numRobots));
for idx = 1:numRobots
    blkName = [rangeSub '/From' num2str(idx)];
    blkPos = [0 30 200 50] + (idx-1)*[0 30 0 30];
    add_block('simulink/Signal Routing/From',blkName);
    set_param(blkName,'GotoTag',['slMultiRobotEnv_Ranges_' num2str(idx)]);
    set_param(blkName,'Position',blkPos);
    add_line(rangeSub,['From' num2str(idx) '/1'],['Vector Concatenate/' num2str(idx)]);
end

% Initialize range sensor bus
internal.createRangeSensorBus;

% Configure object detector tags
objDetSub = [gcb '/ObjectDetectorParams'];
objDetBlocks = find_system(objDetSub,'FollowLinks','on','LookUnderMasks','on','BlockType','From');
for idx = 1:numel(objDetBlocks)
    lh = get_param(objDetBlocks{idx},'LineHandles');
    delete_line(lh.Outport);
end
delete_block(objDetBlocks);
vecBlock = [objDetSub '/Vector Concatenate'];
set_param(vecBlock,'NumInputs',num2str(numRobots));
for idx = 1:numRobots
    blkName = [objDetSub '/From' num2str(idx)];
    blkPos = [0 30 200 50] + (idx-1)*[0 30 0 30];
    add_block('simulink/Signal Routing/From',blkName);
    set_param(blkName,'GotoTag',['slMultiRobotEnv_ObjDet_' num2str(idx)]);
    set_param(blkName,'Position',blkPos);
    add_line(objDetSub,['From' num2str(idx) '/1'],['Vector Concatenate/' num2str(idx)]);
end

% Configure robot detector tags
robotDetSub = [gcb '/RobotDetectorParams'];
robotDetBlocks = find_system(robotDetSub,'FollowLinks','on','LookUnderMasks','on','BlockType','From');
for idx = 1:numel(robotDetBlocks)
    lh = get_param(robotDetBlocks{idx},'LineHandles');
    delete_line(lh.Outport);
end
delete_block(robotDetBlocks);
vecBlock = [robotDetSub '/Vector Concatenate'];
set_param(vecBlock,'NumInputs',num2str(numRobots));
for idx = 1:numRobots
    blkName = [robotDetSub '/From' num2str(idx)];
    blkPos = [0 30 200 50] + (idx-1)*[0 30 0 30];
    add_block('simulink/Signal Routing/From',blkName);
    set_param(blkName,'GotoTag',['slMultiRobotEnv_RobotDet_' num2str(idx)]);
    set_param(blkName,'Position',blkPos);
    add_line(robotDetSub,['From' num2str(idx) '/1'],['Vector Concatenate/' num2str(idx)]);
end

% Configure waypoints port
hasWaypointPort = ~isempty(find_system(gcb,'FollowLinks','on','LookUnderMasks','on','BlockType','Inport','Name','waypoints'));
if ~showWaypoints & hasWaypointPort
    replace_block(gcb,'FollowLinks','on','Name','waypoints','built-in/Ground','noprompt');
elseif showWaypoints & ~hasWaypointPort
    replace_block(gcb,'FollowLinks','on','Name','waypoints','built-in/Inport','noprompt'); 
end
 
% Configure objects port
hasObjectPort = ~isempty(find_system(gcb,'FollowLinks','on','LookUnderMasks','on','BlockType','Inport','Name','objects'));
if ~showObjects & hasObjectPort
    replace_block(gcb,'FollowLinks','on','Name','objects','built-in/Ground','noprompt');
elseif showObjects & ~hasObjectPort
    replace_block(gcb,'FollowLinks','on','Name','objects','built-in/Inport','noprompt');  
end

% Ensure the waypoints and objects ports are organized correctly
if strcmp(get_param([gcb '/waypoints'],'BlockType'),'Inport')
    set_param([gcb '/waypoints'],'Port','1');
end