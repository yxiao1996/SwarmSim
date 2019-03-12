classdef BehaviorBasedSimulation < simulation
    %BEHAVIORBASEDSIMULATION 
    % A simulation for robots navigation using behavior-based method
    
    properties
    end
    
    methods
        function obj = BehaviorBasedSimulation(map,swarmInfo)
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
            % setup environment physics
            obj.physics = AABB(map,swarmInfo.numRobots,0.25,true);
            obj.prev_poses = swarmInfo.poses;
        end
        
        function obj = step(obj)
            readings = obj.sensor_phase();
            controls = obj.control_phase(readings);
            poses = obj.actuate_phase_(controls);
            obj = obj.physics_phase(poses);
            obj.visualize_();
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
    end
end

