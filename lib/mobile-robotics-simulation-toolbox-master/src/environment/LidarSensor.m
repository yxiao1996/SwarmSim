classdef LidarSensor < matlab.System & matlab.system.mixin.CustomIcon & matlab.system.mixin.Propagates
    % LIDARSENSOR 2D Lidar simulator for single-robot environment
    %
    % Returns the angles and ranges of a simulated lidar sensor based on
    % the input map (occupancy grid), scan offsets/angles, and maximum range.
    % Range values outside the maximum range will return NaN.
    %
    % For more information, see <a href="matlab:edit mrsDocLidarSensor">the documentation page</a>
    %
    % Copyright 2018-2019 The MathWorks, Inc.

    %% PROPERTIES
    % Public (user-visible) properties
    properties(Nontunable)
        mapName = ''; % Map
    end
    properties
        sensorOffset = [0,0];           % Lidar sensor offset (x,y) [m]
        scanAngles = [-pi/4,0,pi/4];    % Scan angles [rad]
        maxRange = 5;                   % Maximum range [m]
    end
    
    % Private properties
    properties(Access = private)
        map;     % Occupancy grid
    end

    %% METHODS
    methods(Access = protected)
        
        % Setup method: Initializes all necessary graphics objects
        function setupImpl(obj)         
            % Load the occupancy grid
            obj.map = internal.createMapFromName(obj.mapName);          
        end

        % Step method: Outputs simulated lidar ranges based on map,
        % robot pose, and scan angles/max range
        function ranges = stepImpl(obj,pose)       
            
            % Initialize the ranges to the invalid value (NaN)
            ranges = nan(numel(obj.scanAngles),1);
            
            % If the map is empty or the pose is outside the map limits,
            % return all NaN
            if isempty(obj.map)
                warning('Robot map is empty. Returning NaN for lidar ranges.')
                return;
            elseif (pose(1)<obj.map.XWorldLimits(1)) || (pose(1)>obj.map.XWorldLimits(2)) || ...
                   (pose(2)<obj.map.YWorldLimits(1)) || (pose(2)>obj.map.YWorldLimits(2))
                warning('Robot pose outside world limits. Returning NaN for lidar ranges.');
                return;
            end

            % Find the sensor pose
            theta = pose(3);
            offsetVec = [cos(theta) -sin(theta); ...
                         sin(theta)  cos(theta)]*obj.sensorOffset';
            sensorLoc = pose(1:2)' + offsetVec';                                 

            % Cast a ray from the robot pose through the max range

            % If there is a single sensor offset, use that value and find
            % the intersection points/ranges in a single command
            if size(sensorLoc,1) == 1
               sensorPose = [sensorLoc, theta];
               interPts = rayIntersection(obj.map, sensorPose, ...
                                          obj.scanAngles, obj.maxRange);
               offsets = interPts-sensorPose(1:2);
               ranges = sqrt(sum(offsets.^2,2));
               
            % Else, loop through each sensor location and find the 
            % intersection points/ranges one by one
            else
               
               
               for idx = 1:numel(ranges)
                    sensorPose = [sensorLoc(idx,:), theta]; 
                    interPts = rayIntersection(obj.map, sensorPose, ...
                                    obj.scanAngles(idx), obj.maxRange);
                    offsets = interPts-sensorPose(1:2);
                    ranges(idx) = sqrt(sum(offsets^2,2));
                    
               end
            end
        end         

        % More methods needed for the Simulink block to inherit its output
        % sizes from the scan angle parameter provided.
        function sz = getOutputSizeImpl(obj)
            sz = numel(obj.scanAngles);
        end
        
        function fx = isOutputFixedSizeImpl(~)
           fx = true;
        end
        
        function dt = getOutputDataTypeImpl(obj)
            dt = propagatedInputDataType(obj,1);
        end

        function cp = isOutputComplexImpl(~)
            cp = false;
        end
        
        % Define icon for System block
        function icon = getIconImpl(~)
            icon = {'Lidar','Sensor'};
        end       
        
        % Save and load object implementations
        function s = saveObjectImpl(obj)
            s = saveObjectImpl@matlab.System(obj);
            s.map = obj.map;
        end      
        function loadObjectImpl(obj,s,wasInUse)
            obj.map = s.map;            
            loadObjectImpl@matlab.System(obj,s,wasInUse);
        end
        
    end
    
    methods (Static, Access = protected)
        % Do not show "Simulate using" option
        function flag = showSimulateUsingImpl
            flag = false;
        end
        % Always run in interpreted mode
        function simMode = getSimulateUsingImpl
            simMode = 'Interpreted execution';
        end
    end
    
end
