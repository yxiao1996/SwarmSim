classdef MultiRobotEnv < matlab.System
    % MULTIROBOTENV 2D Multi-Robot Environment
    %
    % Displays the pose (position and orientation) of multiple objects in a 
    % 2D environment. Additionally has the option to display a map as a 
    % robotics.OccupancyGrid or robotics.BinaryOccupancyGrid, object
    % trajectories, waypoints, lidar scans, and/or objects.
    %
    % For more information, see <a href="matlab:edit mrsDocMultiRobotEnv">the documentation page</a>
    %
    % Copyright 2018-2019 The MathWorks, Inc.

    %% PROPERTIES
    % Public (user-visible) properties
    properties(Nontunable)
        numRobots = 1;              % Number of robots
        robotRadius = 0;            % Robot radii [m]
        showTrajectory = false;     % Show trajectory
        mapName = '';               % Map
        hasWaypoints = false;       % Accept waypoints
        hasLidar = false;           % Accept lidar inputs
        hasObjects = false;         % Accept objects
        hasObjDetector = false;     % Accept object detections
        hasRobotDetector = false;   % Accept robot detections
        plotSensorLines = true;     % Plot sensor lines
        showRobotIds = true;        % Show robot IDs
        robotColors = [];           % Robot colors
        % Lidar
        sensorOffset = {[0 0]};          % Lidar sensor offset (x,y) [m]
        scanAngles = {[-pi/4,0,pi/4]};   % Scan angles [rad]
        % Object detectors
        objDetectorOffset = {[0 0]};     % Object detector offset (x,y) [m]
        objDetectorAngle = 0;            % Object detector angle [rad]
        objDetectorFOV = pi/4;           % Object detector field of view [rad]
        objDetectorMaxRange = 5;         % Object detector maximum range [m]
        objectColors = [1 0 0;0 1 0;0 0 1]; % Object label colors [RGB rows]
        objectMarkers = 's';             % Object markers [character array]
        % Robot detectors
        robotDetectorOffset = {[0 0]};   % Robot detector offset (x,y) [m]
        robotDetectorAngle = 0;          % Robot detector angle [rad]
        robotDetectorFOV = pi/4;         % Robot detector field of view [rad]
        robotDetectorMaxRange = 5;       % Robot detector maximum range [m]
    end
    properties % Tunable
        Poses; % Robot poses (x,y,theta) [m,m,rad] 
    end

    % Private properties
    properties(Access = private)
        map;                        % Occupancy grid representing the map
        hasMap;                     % Binary flag for having a map
        fig;                        % Figure window
        ax;                         % Axes for plotting
        RobotHandle;                % Handle to robot body marker or circle
        OrientationHandle;          % Handle to robot orientation line
        LidarHandles;               % Handle array to lidar lines
        TrajHandle;                 % Handle to trajectory plot
        trajX;                      % X Trajectory points
        trajY;                      % Y Trajectory points
        WaypointHandle;             % Handle to waypoints
        ObjectHandles;               % Handle to objects
        ObjDetectorHandles = {};    % Handle array to object detector lines
        RobotDetectorHandles = {};  % Handle array to robot detector lines
        IdHandles;                  % Handle array to robot IDs
    end

    %% METHODS
    methods    
        % Constructor: Takes number of robots as mandatory argument
        function obj = MultiRobotEnv(N)
            obj.numRobots = N;
        end        
    end
        
    methods(Access = protected)
        
        % Setup method: Initializes all necessary graphics objects
        function setupImpl(obj)                 
            % Setup the visualization
            setupVisualization(obj);                
        end

        % Step method: Updates visualization based on inputs
        function stepImpl(obj,robotIndices,poses,varargin)

            % Check for closed figure
            if ~isvalid(obj.fig)
                setupVisualization(obj);
            end            
            
            % Unpack the optional arguments
            idx = 1;
            % Waypoints
            if obj.hasWaypoints
                waypoints = varargin{idx};
                idx = idx + 1;
            else
                waypoints = [];
            end
            % Lidar ranges
            if any(obj.hasLidar) 
                ranges = varargin{idx};
                idx = idx + 1;
            else
                ranges = cell(1,obj.numRobots);
            end
            % Objects
            if numel(varargin) >= idx
                objects = varargin{idx};
            else
                objects = [];
            end
            
            % Draw the waypoints and objects
            drawWaypointsAndObjects(obj,waypoints,objects);
           
            % Draw the robots
            if ~isempty(robotIndices)
                drawRobots(obj,robotIndices,poses,ranges);
            end
            
            % Update the figure
            drawnow('limitrate')           
        end    
        
    end
    
    methods (Access = public) 
        
        % Performs all the visualization setup. It is separate from the
        % setup method, since it can be called independently as well.
        function setupVisualization(obj)
            % Convert scalar flags to arrays based on number of robots
            if numel(obj.robotRadius) ~= obj.numRobots
                obj.robotRadius = repmat(obj.robotRadius,[1,obj.numRobots]);
            end
            if numel(obj.showTrajectory) ~= obj.numRobots
                obj.showTrajectory = repmat(obj.showTrajectory,[1,obj.numRobots]);
            end
            if numel(obj.hasLidar) ~= obj.numRobots
                obj.hasLidar = repmat(obj.hasLidar,[1,obj.numRobots]);
            end
            if numel(obj.hasObjDetector) ~= obj.numRobots
                obj.hasObjDetector = repmat(obj.hasObjDetector,[1,obj.numRobots]);
            end
            if numel(obj.hasRobotDetector) ~= obj.numRobots
                obj.hasRobotDetector = repmat(obj.hasRobotDetector,[1,obj.numRobots]);
            end
            
            % Initialize poses
            obj.Poses = nan(3,obj.numRobots);
            
            % Create figure
            FigureName = 'Multi-Robot Environment';
            FigureTag = 'MultiRobotEnvironment';
            existingFigures = findobj('type','figure','tag',FigureTag);
            if ~isempty(existingFigures)
                obj.fig = figure(existingFigures(1)); % bring figure to the front
                clf;
            else
                obj.fig = figure('Name',FigureName,'tag',FigureTag);
            end
            
            % Create global axes
            obj.ax = axes('parent',obj.fig);
            hold(obj.ax,'on');
            
            % Show the map
            obj.map = internal.createMapFromName(obj.mapName);
            obj.hasMap = ~isempty(obj.map);
            if obj.hasMap
                show(obj.map,'Parent',obj.ax);
            end
            
            % Initialize robot plot
            obj.OrientationHandle = cell(obj.numRobots,1);
            obj.RobotHandle = cell(obj.numRobots,1);
            if isempty(obj.robotColors)
                obj.robotColors = [0 0 1];
            end
            for rIdx = 1:obj.numRobots
                if size(obj.robotColors,1) == 1
                    rColor = obj.robotColors;
                else
                    rColor = obj.robotColors(rIdx,:); 
                end
                obj.OrientationHandle{rIdx} = plot(obj.ax,0,0,'Color',rColor,'LineWidth',1.5);
                if obj.robotRadius(rIdx) > 0
                    % Finite size robot
                    [x,y] = internal.circlePoints(0,0,obj.robotRadius(rIdx),17);
                    obj.RobotHandle{rIdx} = plot(obj.ax,x,y,'Color',rColor,'LineWidth',1.5);
                else
                    % Point robot
                    obj.RobotHandle{rIdx} = plot(obj.ax,0,0,'o','Color',rColor, ...
                         'LineWidth',1.5,'MarkerFaceColor',[1 1 1]);
                end
                % Initialize robot IDs, if enabled
                if obj.showRobotIds
                    obj.IdHandles{rIdx} = text(0,0,num2str(rIdx), ... 
                               'Color',rColor,'FontWeight','bold');
                end
            end
            
            % Initialize trajectory
            obj.TrajHandle = cell(obj.numRobots,1);
            obj.trajX = cell(obj.numRobots,1);
            obj.trajY = cell(obj.numRobots,1);
            for rIdx = 1:obj.numRobots
                if obj.showTrajectory(rIdx)
                    obj.TrajHandle{rIdx} = plot(obj.ax,0,0,'b.-');
                    obj.trajX{rIdx} = [];
                    obj.trajY{rIdx} = [];
                end
            end
            
            % Initialize waypoints
            if obj.hasWaypoints
                obj.WaypointHandle = plot(obj.ax,0,0, ...
                    'rx','MarkerSize',10,'LineWidth',2);
            end
            
            % Initialize lidar lines
            obj.LidarHandles = cell(obj.numRobots,1);
            for rIdx = 1:obj.numRobots
                if obj.hasLidar(rIdx)
                    for idx = 1:numel(obj.scanAngles{rIdx})
                        obj.LidarHandles{rIdx}(idx) = plot(obj.ax,0,0,'b--');
                    end
                end
            end
            
            % Initialize objects and object detector lines
            if obj.hasObjects
                if numel(obj.objectMarkers) == 1
                    obj.ObjectHandles = scatter(obj.ax,[],[],75,...
                        obj.objectMarkers,'filled','LineWidth',2);
                else
                    for idx = 1:numel(obj.objectMarkers)
                        obj.ObjectHandles(idx) = scatter(obj.ax,[],[],75,...
                            obj.objectMarkers(idx),'filled','LineWidth',2);
                    end
                end
            end
            obj.ObjDetectorHandles = cell(obj.numRobots,1);
            for rIdx = 1:obj.numRobots
                if obj.hasObjDetector(rIdx)
                    objDetectorFormat = 'g-.';
                    obj.ObjDetectorHandles{rIdx}(1) = plot(obj.ax,0,0,objDetectorFormat); % Left line
                    obj.ObjDetectorHandles{rIdx}(2) = plot(obj.ax,0,0,objDetectorFormat); % Right line
                end
            end
            

            % Initialize robot detector lines
            obj.RobotDetectorHandles = cell(obj.numRobots,1);
            for rIdx = 1:obj.numRobots
                    robotDetectorFormat = 'm:';
                    obj.RobotDetectorHandles{rIdx} = [ plot(obj.ax,0,0,robotDetectorFormat,'LineWidth',1.5), ... % Left line
                                                       plot(obj.ax,0,0,robotDetectorFormat,'LineWidth',1.5) ];    % Right line
            end
            
            % Final setup
            title(obj.ax,'Multi-Robot Visualization');
            hold(obj.ax,'off');
            axis equal
            
        end
        
        % Helper method to draw the waypoints and objects,
        % which are independent of the robot
        function drawWaypointsAndObjects(obj,waypoints,objects)
            % Check for closed figure
            if ~isvalid(obj.fig)
                return;
            end
            
            % Update waypoints
            if obj.hasWaypoints && (numel(waypoints) > 1)
                set(obj.WaypointHandle,'xdata',waypoints(:,1), ...
                                       'ydata',waypoints(:,2));
            else
                set(obj.WaypointHandle,'xdata',[], ...
                                       'ydata',[]);
            end
            
            % Update the objects
            if size(objects,1) <= 1
                for idx = 1:numel(obj.ObjectHandles)
                    set(obj.ObjectHandles(idx),'xdata',[],'ydata',[],'cdata',[]);
                end
            else
                % Find the corresponding colors of each object
                if size(obj.objectColors,1) == 1
                    colorData = obj.objectColors; % Use the single color specified
                else
                    colorData = obj.objectColors(objects(:,3),:); % Use the color based on labels
                end

                % If single marker, plot markers in the single object handle
                if numel(obj.objectMarkers) == 1

                    set(obj.ObjectHandles,'xdata',objects(:,1),... 
                        'ydata',objects(:,2),'cdata',colorData);
                % If multiple markers, plot markers in the single object handle
                else
                    for idx = 1:numel(obj.objectMarkers)
                        indices = (objects(:,3) == idx);
                        set(obj.ObjectHandles(idx),'xdata',objects(indices,1),... 
                            'ydata',objects(indices,2),'cdata',colorData(indices,:));
                    end
                end
            end
 
        end
        
        % Helper method to draw all robots (calls drawRobot)
        function drawRobots(obj,robotIndices,poses,ranges)
            % Check for closed figure
            if ~isvalid(obj.fig)
                return;
            end
            
            % Draw each robot and its sensors              
            % Single-robot case
            if numel(robotIndices) == 1
               obj.Poses(:,robotIndices) = poses;
               drawRobot(obj,robotIndices,poses,ranges); 
            % Multi-robot case
            else
                obj.Poses = poses;
                for rIdx = robotIndices
                    pose = poses(:,rIdx);
                    drawRobot(obj,rIdx,pose,ranges{rIdx}); 
                end
            end
        end
        
        % Helper method to draw the robot and its sensors at each step
        function drawRobot(obj,rIdx,pose,ranges)
            % Unpack the pose input into (x, y, theta)
            x = pose(1);
            y = pose(2);
            theta = pose(3);
            
            % Update the trajectory
            if obj.showTrajectory(rIdx)
                obj.trajX{rIdx}(end+1) = x;
                obj.trajY{rIdx}(end+1) = y;
                set(obj.TrajHandle{rIdx},'xdata',obj.trajX{rIdx}, ...
                    'ydata',obj.trajY{rIdx});
            end
            
            % Update the robot pose
            xAxesLim = get(obj.ax,'XLim');
            lineLength = diff(xAxesLim)/20;
            r = obj.robotRadius(rIdx);
            if r > 0
                % Finite radius case
                [xc,yc] = internal.circlePoints(x,y,r,17);
                set(obj.RobotHandle{rIdx},'xdata',xc,'ydata',yc);
                len = max(lineLength,2*r); % Plot orientation based on radius unless it's too small
                xp = [x, x+(len*cos(theta))];
                yp = [y, y+(len*sin(theta))];
                set(obj.OrientationHandle{rIdx},'xdata',xp,'ydata',yp);
            else
                % Point robot case
                xp = [x, x+(lineLength*cos(theta))];
                yp = [y, y+(lineLength*sin(theta))];
                set(obj.RobotHandle{rIdx},'xdata',x,'ydata',y);
                set(obj.OrientationHandle{rIdx},'xdata',xp,'ydata',yp);
            end
            % Show robot IDs, if enabled
            if obj.showRobotIds
                set(obj.IdHandles{rIdx},'Position',[x y] - max(1.25*r,lineLength*0.5));
            end
            
            % Update lidar lines
            if obj.hasLidar(rIdx) && obj.plotSensorLines
                
                scanAngs = obj.scanAngles{rIdx};
                
                % Find the sensor pose and check if valid
                offsetVec = [cos(theta) -sin(theta); ...
                             sin(theta)  cos(theta)]*obj.sensorOffset{rIdx}';
                sensorPose = pose + [offsetVec;0];  
                validPoses = true(1,numel(ranges));
                if obj.hasMap
                    % If there is a single sensor offset, use that value
                    if size(sensorPose,2) == 1
                        validPoses(1:numel(ranges)) = ...
                            sensorPose(1) >= obj.map.XWorldLimits(1) && ...
                            sensorPose(1) <= obj.map.XWorldLimits(2) && ... 
                            sensorPose(2) >= obj.map.YWorldLimits(1) && ... 
                            sensorPose(2) <= obj.map.YWorldLimits(2);
                    else
                        for idx = 1:numel(ranges)
                            validPoses(idx) = sensorPose(1,idx) >= obj.map.XWorldLimits(1) && ...
                                              sensorPose(1,idx) <= obj.map.XWorldLimits(2) && ... 
                                              sensorPose(2,idx) >= obj.map.YWorldLimits(1) && ... 
                                              sensorPose(2,idx) <= obj.map.YWorldLimits(2);    
                        end
                    end
                end
               
                for idx = 1:numel(ranges)
                    if validPoses(idx) && ~isnan(ranges(idx))
                        alpha = theta + scanAngs(idx);
                        lidarX = sensorPose(1) + [0, ranges(idx)*cos(alpha)];
                        lidarY = sensorPose(2) + [0, ranges(idx)*sin(alpha)];
                        set(obj.LidarHandles{rIdx}(idx), ...
                            'xdata',lidarX,'ydata',lidarY);
                    else
                        set(obj.LidarHandles{rIdx}(idx),'xdata',[],'ydata',[]);
                    end
                end
            end
            
            % Update object and object detector lines
            if obj.hasObjDetector(rIdx) && obj.plotSensorLines    
                % Find the sensor pose and check if valid
                offsetVec = [cos(theta) -sin(theta); ...
                             sin(theta)  cos(theta)]*obj.objDetectorOffset{rIdx}';
                sensorPose = pose + [offsetVec; obj.objDetectorAngle(rIdx)];  
                if ~obj.hasMap
                    validPose = true;
                else
                    validPose = sensorPose(1) >= obj.map.XWorldLimits(1) && ...
                                sensorPose(1) <= obj.map.XWorldLimits(2) && ... 
                                sensorPose(2) >= obj.map.YWorldLimits(1) && ... 
                                sensorPose(2) <= obj.map.YWorldLimits(2);
                end

                % Plot the object detector lines                
                if validPose
                    maxRange = obj.objDetectorMaxRange(rIdx);
                    % Left line
                    alphaLeft = theta + obj.objDetectorAngle(rIdx) + obj.objDetectorFOV(rIdx)/2;
                    if obj.hasMap
                        intPtsLeft = rayIntersection(obj.map,[sensorPose(1:2)' alphaLeft],0,maxRange);
                    else
                        intPtsLeft = NaN;
                    end
                    if ~isnan(intPtsLeft)
                        objLeftX = [sensorPose(1) intPtsLeft(1)];
                        objLeftY = [sensorPose(2) intPtsLeft(2)];
                    else
                        objLeftX = sensorPose(1) + [0, maxRange*cos(alphaLeft)];
                        objLeftY = sensorPose(2) + [0, maxRange*sin(alphaLeft)];
                    end
                    set(obj.ObjDetectorHandles{rIdx}(1), ...
                        'xdata',objLeftX,'ydata',objLeftY);
                    % Right line
                    alphaRight = theta + obj.objDetectorAngle(rIdx) - obj.objDetectorFOV(rIdx)/2;
                    if obj.hasMap
                        intPtsRight = rayIntersection(obj.map,[sensorPose(1:2)' alphaRight],0,maxRange);
                    else
                        intPtsRight = NaN;
                    end
                    if ~isnan(intPtsRight)
                        objRightX = [sensorPose(1) intPtsRight(1)];
                        objRightY = [sensorPose(2) intPtsRight(2)];
                    else
                        objRightX = sensorPose(1) + [0, maxRange*cos(alphaRight)];
                        objRightY = sensorPose(2) + [0, maxRange*sin(alphaRight)];
                    end
                    set(obj.ObjDetectorHandles{rIdx}(2), ...
                        'xdata',objRightX,'ydata',objRightY);
                end
            end
            
            % Update robot detector lines
            if obj.hasRobotDetector(rIdx) && obj.plotSensorLines  
                % Find the sensor pose and check if valid
                offsetVec = [cos(theta) -sin(theta); ...
                             sin(theta)  cos(theta)]*obj.robotDetectorOffset{rIdx}';
                sensorPose = pose + [offsetVec; obj.robotDetectorAngle(rIdx)];  
                if ~obj.hasMap
                    validPose = true;
                else
                    validPose = sensorPose(1) >= obj.map.XWorldLimits(1) && ...
                                sensorPose(1) <= obj.map.XWorldLimits(2) && ... 
                                sensorPose(2) >= obj.map.YWorldLimits(1) && ... 
                                sensorPose(2) <= obj.map.YWorldLimits(2);
                end
                  
                % Plot the robot detector lines
                if validPose 
                    maxRange = obj.robotDetectorMaxRange(rIdx);
                    % Left line
                    alphaLeft = sensorPose(3) + obj.robotDetectorFOV(rIdx)/2;
                    if obj.hasMap
                        intPtsLeft = rayIntersection(obj.map,[sensorPose(1:2)' alphaLeft],0,maxRange);
                    else
                        intPtsLeft = NaN;
                    end
                    if ~isnan(intPtsLeft)
                        objLeftX = [sensorPose(1) intPtsLeft(1)];
                        objLeftY = [sensorPose(2) intPtsLeft(2)];
                    else
                        objLeftX = sensorPose(1) + [0, maxRange*cos(alphaLeft)];
                        objLeftY = sensorPose(2) + [0, maxRange*sin(alphaLeft)];
                    end
                    set(obj.RobotDetectorHandles{rIdx}(1), ...
                        'xdata',objLeftX,'ydata',objLeftY);
                    % Right line
                    alphaRight = sensorPose(3) - obj.robotDetectorFOV(rIdx)/2;
                    if obj.hasMap
                        intPtsRight = rayIntersection(obj.map,[sensorPose(1:2)' alphaRight],0,maxRange);
                    else
                        intPtsRight = NaN;
                    end
                    if ~isnan(intPtsRight)
                        objRightX = [sensorPose(1) intPtsRight(1)];
                        objRightY = [sensorPose(2) intPtsRight(2)];
                    else
                        objRightX = sensorPose(1) + [0, maxRange*cos(alphaRight)];
                        objRightY = sensorPose(2) + [0, maxRange*sin(alphaRight)];
                    end
                    set(obj.RobotDetectorHandles{rIdx}(2), ...
                        'xdata',objRightX,'ydata',objRightY);
                end
            end            
            
            
        end    
        
        % Attaches all properties associated with a MultiRobotLidarSensor object
        function attachLidarSensor(obj,lidar)
            release(obj)
                     
            if numel(obj.hasLidar) ~= obj.numRobots
                obj.hasLidar(obj.numRobots) = false;
            end
            
            rIdx = lidar.robotIdx; 
            obj.hasLidar(rIdx) = true;
            obj.sensorOffset{rIdx} = lidar.sensorOffset;
            obj.scanAngles{rIdx} = lidar.scanAngles;
            
            % Ensure to use the same map as the visualizer
            setEnvironment(lidar,obj);
        end
        
        % Attaches all properties associated with an ObjectDetector object
        function attachObjectDetector(obj,rIdx,detector)
            if numel(obj.hasObjDetector) ~= obj.numRobots
                obj.hasObjDetector(obj.numRobots) = false;
            end 
            obj.hasObjects = true;
            obj.hasObjDetector(rIdx) = true;
            obj.objDetectorOffset{rIdx} = detector.sensorOffset;
            obj.objDetectorAngle(rIdx) = detector.sensorAngle;
            obj.objDetectorFOV(rIdx) = detector.fieldOfView;
            obj.objDetectorMaxRange(rIdx) = detector.maxRange;
            
            % Ensure to use the same map as the visualizer
            release(detector);
            detector.mapName = obj.mapName;
        end

        % Attaches all properties associated with a RobotDetector object
        function attachRobotDetector(obj,detector)
            release(obj);
            
            if numel(obj.hasRobotDetector) ~= obj.numRobots
                obj.hasRobotDetector(obj.numRobots) = false;
            end
            
            rIdx = detector.robotIdx;
            obj.hasRobotDetector(rIdx) = true;
            obj.robotDetectorOffset{rIdx} = detector.sensorOffset;
            obj.robotDetectorAngle(rIdx) = detector.sensorAngle;
            obj.robotDetectorFOV(rIdx) = detector.fieldOfView;
            obj.robotDetectorMaxRange(rIdx) = detector.maxRange;
            
        end
        
    end
    
    methods (Access = protected)
        
        % Define total number of inputs for system with optional inputs
        function n = getNumInputsImpl(obj)
            n = 2;
            if obj.hasWaypoints
                n = n + 1;
            end
            if any(obj.hasLidar) 
                n = n + 1;
            end
            if obj.hasObjects
                n = n + 1;
            end 
        end 
    end
        
end