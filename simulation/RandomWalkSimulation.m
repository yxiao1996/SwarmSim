classdef RandomWalkSimulation < simulation
    %SIMULATOR discrete-time simulator for multi-robot system
    
    properties
        %prev_poses
    end
    
    methods
        function obj = RandomWalkSimulation(map,params)
            R = 0.1; L = 0.5;
            obj.sampleTime = 0.05;
            obj.numRobots = params.numRobots;
            obj.world = world(params); 
            obj.controllers = cell(1,obj.numRobots);
            for i = 1:obj.numRobots
                obj.controllers{i} = RandomWalkDiffDrive([0.5 1],[-pi/2,pi/2]);
            end
            obj.actuators = cell(1,obj.numRobots);
            for i = 1:obj.numRobots
                obj.actuators{i} = actuatorDiffDrive(R,L);
            end
            obj.physics = AABB(map,params.numRobots,0.25,true);
            obj.prev_poses = params.initial_poses;
        end
        
        function obj = step_(obj) % single part implementation
           %% main simulation step function
            poses = obj.world.get_poses(); % current system states
            % sensor reading phase
            readings = obj.world.readSensors();
            % control phase
            controls = cell(1,obj.numRobots);
            for i = 1:obj.numRobots
                ctl = obj.controllers{i};
                controls{i} = ctl.compute_control(poses,readings);
            end
            % actuate phase
            for i = 1:obj.numRobots
                actuator = obj.actuators{i};
                pose = poses(:,i);
                control = controls{i};
                vel = actuator.actuate(control,pose);
                poses(:,i) = pose  + vel * obj.sampleTime;
            end
            % physics phase
            poses = obj.physics.check_robots(poses,obj.prev_poses);
            poses = obj.physics.check_obstacles(poses,obj.prev_poses);
            obj.world = obj.world.update_poses(poses);
            obj.prev_poses = poses;
            % update visualization
            readings = obj.world.readSensors();
            obj.world.visualize(readings);
        end
        
        function obj = step(obj)
            readings = obj.sensor_phase();
            controls = obj.control_phase(readings);
            poses = obj.actuate_phase(controls);
            obj = obj.physics_phase(poses);
            obj.visualize();
        end
        
        %function readings = sensor_phase(obj)
        %    readings = obj.world.readSensors();
        %end
        
        function controls = control_phase(obj,readings)
            poses = obj.world.get_poses(); % current system states
            controls = cell(1,obj.numRobots);
            for i = 1:obj.numRobots
                ctl = obj.controllers{i};
                controls{i} = ctl.compute_control(poses,readings);
            end
        end
        
        function poses = actuate_phase(obj,controls)
            poses = obj.world.get_poses(); % current system states
            for i = 1:obj.numRobots
                actuator = obj.actuators{i};
                pose = poses(:,i);
                control = controls{i};
                vel = actuator.actuate(control,pose);
                poses(:,i) = pose  + vel * obj.sampleTime;
            end
        end
        
        %function obj = physics_phase(obj,poses)
        %    poses = obj.physics.check_obstacles(poses,obj.prev_poses);
        %    obj.world = obj.world.update_poses(poses);
        %    obj.prev_poses = poses;
        %end
        
        function visualize(obj)
            readings = obj.world.readSensors();
            obj.world.visualize(readings);
        end
    end
end

