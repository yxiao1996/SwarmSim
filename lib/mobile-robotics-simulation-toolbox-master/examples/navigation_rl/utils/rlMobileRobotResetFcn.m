function in = rlMobileRobotResetFcn(in,scanAngles,maxRange,mapName)
% Reset function for reinforcement learning based obstacle avoidance
% Copyright 2019 The MathWorks, Inc.

    % Load map and lidar sensor (to generate valid pose)
    persistent map lidar
    if isempty(map)
        s = load(mapName);
        map = s.map;
        lidar = LidarSensor;
        lidar.mapName = 'map';
        lidar.scanAngles = scanAngles;
        lidar.maxRange = maxRange;
    end
    
    % Randomly generate pose inside the map. 
    % If the pose is in an unoccupied space and there are no range readings
    % nearby, assign this pose to the new simulation run
    posFound = false;   
    while(~posFound)
        pos = [diff(map.XWorldLimits)*rand + map.XWorldLimits(1); ...
               diff(map.YWorldLimits)*rand + map.YWorldLimits(1); ... 
               2*pi*rand];  
        ranges = lidar(pos);
        if ~checkOccupancy(map,pos(1:2)') && all(ranges(~isnan(ranges)) >= 0.5)
            posFound = true;
            in = setVariable(in,'initX', pos(1));
            in = setVariable(in,'initY', pos(2));
            in = setVariable(in,'initTheta', pos(3));
        end
    end
    
    % Shuffle the lidar sensor noise seeds
    in = setVariable(in,'lidarNoiseSeeds',randi(intmax,size(lidar.scanAngles)));
        
end

