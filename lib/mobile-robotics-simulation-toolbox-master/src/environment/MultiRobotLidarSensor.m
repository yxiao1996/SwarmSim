classdef MultiRobotLidarSensor < matlab.System & matlab.system.mixin.CustomIcon & matlab.system.mixin.Propagates
    % MULTIROBOTLIDARSENSOR 2D Lidar simulator for multi-robot environment
    %
    % Returns the angles and ranges of a simulated lidar sensor based on
    % the input map (occupancy grid), scan offsets/angles, and maximum range.
    % Range values outside the maximum range will return NaN.
    %
    % For more information, see <a href="matlab:edit mrsDocLidarSensor">the documentation page</a>
    %
    % Copyright 2019 The MathWorks, Inc.
    
    %% PROPERTIES
    % Public (user-visible) properties
    properties(Nontunable)
        mapName = ''; % Map
    end
    properties
        sensorOffset = [0,0];           % Lidar sensor offset (x,y) [m]
        scanAngles = [-pi/4,0,pi/4];    % Scan angles [rad]
        maxRange = 5;                   % Maximum range [m]
        robotIdx = 1;                   % Robot index
        robotRadii = -1;                % Robot radii
    end
    
    % Private properties
    properties(Access = private)
        env;     % MultiRobotEnv object
        lidar;   % Single-robot lidar
    end
    
    %% METHODS
    methods
        
        % Set the environment
        function setEnvironment(obj,env)
            obj.env = env;
        end
        
    end
    
    methods(Access = protected)
        
        % Setup method: Initializes all necessary graphics objects
        function setupImpl(obj)
            
            % Create a LidarSensor with the same properties
            obj.lidar = LidarSensor;
            obj.lidar.sensorOffset = obj.sensorOffset;
            obj.lidar.scanAngles = obj.scanAngles;
            obj.lidar.maxRange = obj.maxRange;
            
            % Attach the robot detector to the environment, if it exists
            if ~isempty(obj.env)
                attachLidarSensor(obj.env,obj);
                % Ensure to use the same map as the visualizer
                obj.mapName = obj.env.mapName;
            end
            
            obj.lidar.mapName = obj.mapName;
            
        end
        
        % Step method: Outputs simulated lidar ranges based on map,
        % robot pose, and scan angles/max range
        function ranges = stepImpl(obj,varargin)
            
            % Find the sensor pose
            if nargin < 2
                targetPoses = obj.env.Poses; % If using an attached environment (MATLAB usage)
                targetRadii = obj.env.robotRadius;
            else
                targetPoses = varargin{1}; % If passing in the poses as input (Simulink usage)
                if nargin > 2 || isempty(obj.robotRadii)
                    targetRadii = varargin{2};    % If the radii are passed in as input (or they have to be)
                else
                    targetRadii = obj.robotRadii; % If the radii are set as parameters
                end
            end
            % Find the sensor pose
            pose = targetPoses(:,obj.robotIdx);
            
            % Step the single-robot lidar
            ranges = obj.lidar(pose);
            
            % Check for additional multi-robot intersections
            theta = pose(3);
            offsetVec = [cos(theta) -sin(theta); ...
                sin(theta)  cos(theta)]*obj.sensorOffset';
            sensorPose = pose + [offsetVec;0];
            for rIdx = 1:numel(targetRadii)
                if (rIdx ~= obj.robotIdx) && (~isempty(targetPoses)) && (targetRadii(rIdx)>0)
                    robotRanges = internal.circleLineIntersection( ...
                        sensorPose,obj.scanAngles,obj.maxRange, ...
                        targetPoses(:,rIdx),targetRadii(rIdx));
                    ranges = min(ranges,robotRanges);
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
        function icon = getIconImpl(obj)
            icon = {'Multi-Robot',['Lidar ' num2str(obj.robotIdx)]};
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
    
    methods (Access = protected)
        % Define total number of inputs for system with optional inputs
        function n = getNumInputsImpl(obj)
            if isempty(obj.env)
                if any(obj.robotRadii < 0)
                    n = 2;
                else
                    n = 1;
                end
            else
                n = 0;
            end
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
