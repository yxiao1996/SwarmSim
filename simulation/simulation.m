classdef simulation
    %SIMULATION Abstract class for a simulation
    
    properties
        sampleTime
        numRobots
        world
        physics
        controllers
        actuators
        prev_poses
        hPlot
    end
    
    methods (Abstract)
        control_phase(obj)
        step(obj)
    end
    
    methods
        function readings = sensor_phase(obj)
            readings = obj.world.readSensors();
        end
        
        function obj = physics_phase(obj,poses)
            poses = obj.physics.check_obstacles(poses,obj.prev_poses);
            %poses = obj.physics.check_robots(poses,obj.prev_poses);
            obj.world = obj.world.update_poses(poses);
            obj.prev_poses = poses;
        end
        
        function poses = actuate_phase_(obj,controls)
            poses = obj.world.get_poses(); % current system states
            for i = 1:obj.numRobots
                actuator = obj.actuators{i};
                pose = poses(:,i);
                control = controls{i};
                vel = actuator.actuate(control,pose);
                poses(:,i) = pose  + vel * obj.sampleTime;
            end
        end
        
        function visualize_(obj)
            readings = obj.world.readSensors();
            obj.world.visualize(readings);
        end
        
        
    end
    
end

