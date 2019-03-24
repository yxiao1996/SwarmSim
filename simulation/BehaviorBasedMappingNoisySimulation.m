classdef BehaviorBasedMappingNoisySimulation < simulation
    %BEHAVIORBASEDSIMULATION 
    % A simulation for robots navigation using behavior-based method
    
    properties
        robotDetectors
        mapper
    end
    
    methods
        function obj = BehaviorBasedMappingNoisySimulation(map,swarmInfo,flt)
            %BEHAVIORBASEDSIMULATION 
            % construct the simulation
            obj.sampleTime = 0.05;
            obj.numRobots = swarmInfo.numRobots;
            obj.world = world(swarmInfo);
            obj.controllers = cell(1,obj.numRobots);
            % assign controller to each robot
            goal = [12 12];
            robotInfos = swarmInfo.infos;
            for i = 1:obj.numRobots
                robotInfo = robotInfos{i};
                obj.controllers{i} = DiffDriveBehaviorBased(robotInfo,goal);
            end
            % assign actuator to each robot
            obj.actuators = cell(1,obj.numRobots);
            for i = 1:obj.numRobots
                robotInfo = swarmInfo.infos{i};
                R = robotInfo.wheel_radius;
                L = robotInfo.body_width;
                if (robotInfo.type == "DiffDrive")
                    obj.actuators{i} = actuatorDiffDrive(R,L);
                end
            end
            % assign robot detectors to each robot
            obj.robotDetectors = cell(1,obj.numRobots);
            for i = 1:obj.numRobots
                detector = RobotDetectors(swarmInfo,i);
                obj.robotDetectors{i} = detector;
            end
            % setup environment physics
            obj.physics = AABB(map,swarmInfo.numRobots,0.25,true);
            obj.prev_poses = swarmInfo.poses;
            % add mapper
            obj.mapper = RangeFinderNoisyMapper(10000,swarmInfo,flt);
        end
        
        function obj = step(obj)
            readings = obj.sensor_phase();
            controls = obj.control_phase(readings);
            poses = obj.actuate_phase_(controls);
            obj = obj.physics_phase(poses);
            obj.visualize_();
            masks = obj.detect_phase();
            readings = obj.sensor_phase();
            %poses = obj.world.get_poses();
            obj = obj.mapping_phase(poses,readings,masks,controls);
        end
        
         function controls = control_phase(obj,readings)
            poses = obj.world.get_poses(); % current system states
            controls = cell(1,obj.numRobots);
            for i = 1:obj.numRobots
                ctl = obj.controllers{i};
                pose = poses(:,i);
                reading = readings{i};
                controls{i} = ctl.compute_control(pose,reading);
            end
         end
         
         function masks = detect_phase(obj)
             detections = obj.world.readDetections();
             masks = cell(1,obj.numRobots);
             for i = 1:obj.numRobots
                 detection = detections{i};
                 detector = obj.robotDetectors{i};
                 mask = detector.sensor_mask(detection);
                 masks{i} = mask;
             end
         end
         
         function obj = mapping_phase(obj,poses,readings,masks,controls)
             for i = 1:obj.numRobots
                reading = readings{i};
                mask = masks{i};
                pose = squeeze(poses(:,i));
                control = controls{i};
                obj.mapper = obj.mapper.addPoints(pose,reading,mask,control,i);
             end
             
         end
    end
end

