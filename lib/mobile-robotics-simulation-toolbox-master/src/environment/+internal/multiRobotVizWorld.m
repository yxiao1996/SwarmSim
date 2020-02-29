function multiRobotVizWorld(numRobots,visuals,poses,waypoints,ranges,objects, ...
    objDetParams,robotDetParams,showRobotIds,plotSensorLines)
% Wrapper function that uses a MultiRobotEnv object from a Simulink model
% that receives all inputs and parameters as function inputs.
%
% Copyright 2019 The MathWorks, Inc.

% Initialize the multi-robot environment
persistent slMultiRobotEnv
if isempty(slMultiRobotEnv)
    slMultiRobotEnv = MultiRobotEnv(numRobots);
    
    % Set the robot visuals and sensor line plots
    slMultiRobotEnv.robotRadius = visuals(1,:);
    slMultiRobotEnv.robotColors = visuals(2:4,:)';
    slMultiRobotEnv.showRobotIds = showRobotIds;
    slMultiRobotEnv.plotSensorLines = plotSensorLines;
    
    % Set the map
    blkName = gcb;
    topBlkName = blkName(1:end-10);
    slMultiRobotEnv.mapName = get_param(topBlkName,'mapName');
    
    % Set the waypoint flag
    slMultiRobotEnv.hasWaypoints = ~isscalar(waypoints);
    
    % Initialize range sensors
    for idx = 1:numRobots
        slMultiRobotEnv.hasLidar(idx) = ranges(idx).hasLidar;
        slMultiRobotEnv.scanAngles{idx} = ranges(idx).scanAngles';
        slMultiRobotEnv.sensorOffset{idx} = ranges(idx).sensorOffset';
    end
    
    % Initialize the objects
    if ~isscalar(objects)
        slMultiRobotEnv.hasObjects = true;
        slMultiRobotEnv.objectColors = evalin('base',get_param(topBlkName,'objectColors'));
        slMultiRobotEnv.objectMarkers = evalin('base',get_param(topBlkName,'objectMarkers'));
    end
    
    % Initialize object detector params
    slMultiRobotEnv.hasObjDetector = objDetParams(1,:)';
    for idx = 1:numRobots
        slMultiRobotEnv.objDetectorOffset{idx} = objDetParams(2:3,idx)';
    end
    slMultiRobotEnv.objDetectorAngle = objDetParams(4,:)';
    slMultiRobotEnv.objDetectorFOV = objDetParams(5,:)';
    slMultiRobotEnv.objDetectorMaxRange = objDetParams(6,:)';
    
    % Initialize robot detector params
    slMultiRobotEnv.hasRobotDetector = robotDetParams(1,:)';
    for idx = 1:numRobots
        slMultiRobotEnv.robotDetectorOffset{idx} = robotDetParams(2:3,idx)';
    end
    slMultiRobotEnv.robotDetectorAngle = robotDetParams(4,:)';
    slMultiRobotEnv.robotDetectorFOV = robotDetParams(5,:)';
    slMultiRobotEnv.robotDetectorMaxRange = robotDetParams(6,:)';
    
end

% Extract the range inputs from the input structures
if any(slMultiRobotEnv.hasLidar)
    if numRobots == 1
        rangeInput = ranges.ranges;
    else
        rangeInput = cell(1,numRobots);
        for idx = 1:numRobots
            rangeInput{idx} = ranges(idx).ranges(1:ranges(idx).numScans);
        end
    end
end

% Step the environment with the correct function signature to update the visualization
if slMultiRobotEnv.hasWaypoints
    if slMultiRobotEnv.hasObjects
        if any(slMultiRobotEnv.hasLidar)
            step(slMultiRobotEnv,1:numRobots,poses,waypoints,rangeInput,objects);
        else
            step(slMultiRobotEnv,1:numRobots,poses,waypoints,objects);
        end
    else
        if any(slMultiRobotEnv.hasLidar)
            step(slMultiRobotEnv,1:numRobots,poses,waypoints,rangeInput);
        else
            step(slMultiRobotEnv,1:numRobots,poses,waypoints);
        end
    end
else
    if slMultiRobotEnv.hasObjects
        if any(slMultiRobotEnv.hasLidar)
            step(slMultiRobotEnv,1:numRobots,poses,rangeInput,objects);
        else
            step(slMultiRobotEnv,1:numRobots,poses,objects);
        end
    else
        if any(slMultiRobotEnv.hasLidar)
            step(slMultiRobotEnv,1:numRobots,poses,rangeInput);
        else
            step(slMultiRobotEnv,1:numRobots,poses);
        end        
    end
end

end