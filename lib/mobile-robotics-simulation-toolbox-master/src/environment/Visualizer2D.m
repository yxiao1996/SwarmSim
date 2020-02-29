classdef Visualizer2D < matlab.System & matlab.system.mixin.CustomIcon
    % VISUALIZER2D 2D Robot Visualizer
    %
    % Displays the pose (position and orientation) of an object in a 2D
    % environment. Additionally has the option to display a map as a 
    % robotics.OccupancyGrid or robotics.BinaryOccupancyGrid, object
    % trajectory, waypoints, lidar scans, and/or objects.
    %
    % For more information, see <a href="matlab:edit mrsDocVisualizer2D">the documentation page</a>
    %
    % Copyright 2018-2019 The MathWorks, Inc.

    %% PROPERTIES
    % Public (user-visible) properties
    properties(Nontunable)
        robotRadius = 0;    % Robot radius [m]
        mapName = '';       % Map
    end     
    properties(Nontunable, Logical)
        showTrajectory = true;      % Show trajectory
        hasWaypoints = false;       % Accept waypoints
        hasLidar = false;           % Accept lidar inputs
    end
    properties
       sensorOffset = [0 0];        % Lidar sensor offset (x,y) [m] 
       scanAngles = [-pi/4,0,pi/4]; % Scan angles [rad]
    end
    properties(Nontunable, Logical)
        hasObjDetector = false;     % Accept object detections
    end
    properties(Nontunable)
       objDetectorOffset = [0 0];   % Object detector offset (x,y) [m] 
       objDetectorAngle = 0;        % Object detector angle [rad]
       objDetectorFOV = pi/4;       % Object detector field of view [rad] 
       objDetectorMaxRange = 5;     % Object detector maximum range [m]
       objectColors = [1 0 0;0 1 0;0 0 1];  % Object label colors [RGB rows]
       objectMarkers = 's';       % Object markers [character array]
    end

    % Private properties
    properties(Access = private)
        map;                % Occupancy grid
        fig;                % Figure window
        ax;                 % Axes for plotting
        RobotHandle;        % Handle to robot body marker or circle
        OrientationHandle;  % Handle to robot orientation line
        LidarHandles;       % Handle array to lidar lines
        TrajHandle;         % Handle to trajectory plot
        trajX = [];         % X Trajectory points
        trajY = [];         % Y Trajectory points
        WaypointHandle;     % Handle to waypoints
        ObjectHandles;      % Handle to objects
        ObjDetectorHandles; % Handle array to object detector lines
    end

    %% METHODS
    methods(Access = protected)
        
        % Setup method: Initializes all necessary graphics objects
        function setupImpl(obj)
            % Create figure
            FigureName = 'Robot Visualization';
            FigureTag = 'RobotVisualization';
            existingFigures = findobj('type','figure','tag',FigureTag);
            if ~isempty(existingFigures)
                obj.fig = figure(existingFigures(1)); % bring figure to the front
                clf;
            else
                obj.fig = figure('Name',FigureName,'tag',FigureTag);
            end
            obj.ax = axes('parent',obj.fig);   
            hold(obj.ax,'on');
            
            % Show the map
            obj.map = internal.createMapFromName(obj.mapName);
            if ~isempty(obj.map)
                show(obj.map,'Parent',obj.ax);
            end
            
            % Initialize robot plot
            obj.OrientationHandle = plot(obj.ax,0,0,'r','LineWidth',1.5);
            if obj.robotRadius > 0
                % Finite size robot
                [x,y] = internal.circlePoints(0,0,obj.robotRadius,17);
                obj.RobotHandle = plot(obj.ax,x,y,'b','LineWidth',1.5);
            else
                % Point robot
                obj.RobotHandle = plot(obj.ax,0,0,'bo', ...
                    'LineWidth',1.5,'MarkerFaceColor',[1 1 1]);
            end
            
            % Initialize trajectory
            if obj.showTrajectory
                obj.TrajHandle = plot(obj.ax,0,0,'b.-');
            end
            
            % Initialize waypoints
            if obj.hasWaypoints
                obj.WaypointHandle = plot(obj.ax,0,0, ...
                   'rx','MarkerSize',10,'LineWidth',2);
            end
            
            % Initialize lidar lines
            if obj.hasLidar
                for idx = 1:numel(obj.scanAngles)
                    obj.LidarHandles(idx) = plot(obj.ax,0,0,'b--');
                end
            end
            
            % Initialize objects and object detector lines
            if obj.hasObjDetector
                if numel(obj.objectMarkers) == 1
                    obj.ObjectHandles = scatter(obj.ax,0,0,75,... 
                        obj.objectMarkers,'filled','LineWidth',2);
                else
                    for idx = 1:numel(obj.objectMarkers)
                        obj.ObjectHandles(idx) = scatter(obj.ax,0,0,75,... 
                            obj.objectMarkers(idx),'filled','LineWidth',2);
                    end
                end
                objDetectorFormat = 'g-.';
                obj.ObjDetectorHandles(1) = plot(obj.ax,0,0,objDetectorFormat); % Left line
                obj.ObjDetectorHandles(2) = plot(obj.ax,0,0,objDetectorFormat); % Right line
            end
            
            % Final setup
            title(obj.ax,'Robot Visualization');
            hold(obj.ax,'off'); 
            axis equal          
        end

        % Step method: Updates visualization based on inputs
        function stepImpl(obj,pose,varargin)          
            % Unpack the pose input into (x, y, theta)
            x = pose(1);
            y = pose(2);
            theta = pose(3);
            
            % Check for closed figure
            if ~isvalid(obj.fig)
                return;
            end
            
            % Unpack the optional arguments
            idx = 1;
            if obj.hasWaypoints % Waypoints
                waypoints = varargin{idx};
                idx = idx + 1;
            end
            if obj.hasLidar % Lidar ranges
                ranges = varargin{idx};
                idx = idx + 1;
            end
            if obj.hasObjDetector % Objects and object detections
                objects = varargin{idx};
            end
                       
            % Update the trajectory
            if obj.showTrajectory
               obj.trajX = [obj.trajX;x];
               obj.trajY = [obj.trajY;y];
               set(obj.TrajHandle,'xdata',obj.trajX,'ydata',obj.trajY);
            end
            
            % Update waypoints
            if obj.hasWaypoints && numel(waypoints) > 1
                set(obj.WaypointHandle,'xdata',waypoints(:,1), ...
                                       'ydata',waypoints(:,2));
            end

            % Update the robot pose
            xAxesLim = get(obj.ax,'XLim');
            lineLength = diff(xAxesLim)/20;
            if obj.robotRadius > 0
                % Finite radius case
                [xc,yc] = internal.circlePoints(x,y,obj.robotRadius,17);
                set(obj.RobotHandle,'xdata',xc,'ydata',yc);
                len = max(lineLength,2*obj.robotRadius); % Plot orientation based on radius unless it's too small
                xp = [x, x+(len*cos(theta))];
                yp = [y, y+(len*sin(theta))];
                set(obj.OrientationHandle,'xdata',xp,'ydata',yp);
            else
                % Point robot case
                xp = [x, x+(lineLength*cos(theta))];
                yp = [y, y+(lineLength*sin(theta))];
                set(obj.RobotHandle,'xdata',x,'ydata',y);
                set(obj.OrientationHandle,'xdata',xp,'ydata',yp);
            end
            
            % Update lidar lines
            if obj.hasLidar
                
                % Find the sensor location(s)
                offsetVec = [cos(theta) -sin(theta); ... 
                             sin(theta)  cos(theta)]*obj.sensorOffset';
                sensorLoc = [x,y] + offsetVec';  
                
                for idx = 1:numel(ranges)
                    if ~isnan(ranges(idx))
                       
                        % If there is a single sensor offset, use that value
                        if size(sensorLoc,1) == 1
                           sensorPos = sensorLoc;
                        % Else, use the specific index's sensor location
                        else
                           sensorPos = sensorLoc(idx,:); 
                        end
                        
                        alpha = theta + obj.scanAngles(idx);                      
                        lidarX = sensorPos(1) + [0, ranges(idx)*cos(alpha)];
                        lidarY = sensorPos(2) + [0, ranges(idx)*sin(alpha)];
                        set(obj.LidarHandles(idx),'xdata',lidarX, ...
                                                  'ydata',lidarY); 
                    else
                        set(obj.LidarHandles(idx),'xdata',[],'ydata',[]); 
                    end 
                end
            end         
            
            % Update object and object detector lines
            if obj.hasObjDetector
                
                if isempty(objects)
                    set(obj.ObjectHandles,'xdata',[],'ydata',[],'cdata',[]);
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
                
                % Find the sensor location(s)
                offsetVec = [cos(theta) -sin(theta); ... 
                             sin(theta)  cos(theta)]*obj.objDetectorOffset';
                sensorLoc = [x,y] + offsetVec';  
                                
                % Plot the object detector lines
                % Left line
                alphaLeft = theta + obj.objDetectorAngle + obj.objDetectorFOV/2;        
                if ~isempty(obj.map) 
                    intPtsLeft = rayIntersection(obj.map,[sensorLoc alphaLeft], ...
                                                 0,obj.objDetectorMaxRange);
                else
                    intPtsLeft = NaN;
                end
                if ~isnan(intPtsLeft)
                    objLeftX = [sensorLoc(1) intPtsLeft(1)];
                    objLeftY = [sensorLoc(2) intPtsLeft(2)];    
                else
                    objLeftX = sensorLoc(1) + [0, obj.objDetectorMaxRange*cos(alphaLeft)];
                    objLeftY = sensorLoc(2) + [0, obj.objDetectorMaxRange*sin(alphaLeft)];   
                end
                set(obj.ObjDetectorHandles(1),'xdata',objLeftX, ...
                                              'ydata',objLeftY); 
                % Right line
                alphaRight = theta + obj.objDetectorAngle - obj.objDetectorFOV/2;        
                if ~isempty(obj.map) 
                    intPtsRight = rayIntersection(obj.map,[sensorLoc alphaRight], ...
                                                  0,obj.objDetectorMaxRange);
                else
                   intPtsRight = NaN; 
                end
                if ~isnan(intPtsRight)
                    objRightX = [sensorLoc(1) intPtsRight(1)];
                    objRightY = [sensorLoc(2) intPtsRight(2)];    
                else
                    objRightX = sensorLoc(1) + [0, obj.objDetectorMaxRange*cos(alphaRight)];
                    objRightY = sensorLoc(2) + [0, obj.objDetectorMaxRange*sin(alphaRight)];   
                end
                set(obj.ObjDetectorHandles(2),'xdata',objRightX, ...
                                              'ydata',objRightY); 
            end
            
            % Update the figure
            drawnow('limitrate')
            
        end

        % Define total number of inputs for system with optional inputs
        function num = getNumInputsImpl(obj)
            num = 1;
            if obj.hasWaypoints
               num = num + 1; 
            end
            if obj.hasLidar
               num = num + 1;
            end
            if obj.hasObjDetector
               num = num + 1; 
            end
        end
        
        % Define input port names
        function [namePose,varargout] = getInputNamesImpl(obj)
            namePose = 'pose';
            idx = 1;
            if obj.hasWaypoints
               varargout{idx} = 'waypoints';
               idx = idx + 1;
            end
            if obj.hasLidar
               varargout{idx} = 'ranges';
               idx = idx + 1;
            end
            if obj.hasObjDetector
               varargout{idx} = 'objects';
            end
        end

        % Define icon for System block
        function icon = getIconImpl(~)
            icon = {'Robot','Visualizer'};
        end
        
        % Inactivate properties if there's no lidar or object detector
        function flag = isInactivePropertyImpl(obj,prop)
             flag = false;
             switch prop
                 case 'sensorOffset'
                     flag = ~obj.hasLidar;
                 case 'scanAngles'
                     flag = ~obj.hasLidar;
                 case 'objDetectorOffset'
                     flag = ~obj.hasObjDetector;
                 case 'objDetectorAngle'
                     flag = ~obj.hasObjDetector;
                 case 'objDetectorFOV'
                     flag = ~obj.hasObjDetector;
                 case 'objDetectorMaxRange'
                     flag = ~obj.hasObjDetector;
                 case 'objectColors'
                     flag = ~obj.hasObjDetector;
                 case 'objectMarkers'
                     flag = ~obj.hasObjDetector;
             end
        end
        
    end
    
    methods (Access = public)        
        % Attaches all properties associated with a LidarSensor object
        function attachLidarSensor(obj,lidar)
            obj.hasLidar = true;
            obj.sensorOffset = lidar.sensorOffset;
            obj.scanAngles = lidar.scanAngles;
            
            % Ensure to use the same map as the visualizer
            release(lidar);
            lidar.mapName = obj.mapName;
        end
        
        % Attaches all properties associated with an ObjectDetector object
        function attachObjectDetector(obj,detector)
            obj.hasObjDetector = true;
            obj.objDetectorOffset = detector.sensorOffset;
            obj.objDetectorAngle = detector.sensorAngle;
            obj.objDetectorFOV = detector.fieldOfView;
            obj.objDetectorMaxRange = detector.maxRange;
            
            % Ensure to use the same map as the visualizer
            release(detector);
            detector.mapName = obj.mapName;
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