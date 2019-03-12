classdef DiffDrivePRMSimulation < simulation
    %DIFFDRIVEPRMSIMULATION 
    %   A simulation of robots navigate using PRM algorithms
    
    properties
    end
    
    methods
        function obj = DiffDrivePRMSimulation(map,swarmInfo)
            %DIFFDRIVEPRMSIMULATION 
            % construct the simulation
            obj.sampleTime = 0.05;
            obj.numRobots = swarmInfo.numRobots;
            obj.world = world(swarmInfo);
            obj.controllers = cell(1,obj.numRobots);
            % assign controller to each robot
            goal = [12 2];
            map_inf = copy(map);
            inflate(map_inf,0.25);
            for i = 1:obj.numRobots
                pose = swarmInfo.poses(:,i);
                waypoints = planPRM(map_inf,pose,goal);
                obj.controllers{i} = DiffDrivePursueWayPoints(waypoints);
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
                controls{i} = ctl.compute_control(pose,readings);
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
        
        function visualize(obj)
            readings = obj.world.readSensors();
            obj.world.visualize(readings);
        end
    end
end

