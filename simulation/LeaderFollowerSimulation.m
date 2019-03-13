classdef LeaderFollowerSimulation < simulation
    %LEADERFOLLOWERSIMULATION 
    % A Leader-Follower formation control simulation
    
    properties
        form
        numFollowers
    end
    
    methods
        function obj = LeaderFollowerSimulation(map,swarmInfo,form)
            %LEADERFOLLOWERSIMULATION 
            obj.sampleTime = 0.05;
            obj.numRobots = swarmInfo.numRobots;
            obj.numFollowers = obj.numRobots - 1;
            obj.world = world(swarmInfo);
            obj.controllers = cell(1,obj.numRobots);
            % assign controller for the leader(PRM waypoints)
            leader_goal = [12 12];
            leader_pose = swarmInfo.poses(:,1);
            leader_wps = planPRM(map,leader_pose,leader_goal);
            obj.controllers{1} = DiffDrivePursueWayPoints(leader_wps);
            % assign controller for the followers
            obj.form = form;%LineFormation();
            %type="dphi"; params.d = 1; params.phi = 0;
            for i = 2:obj.numRobots
                type = obj.form.getType(i);
                %leadIdx = form.getIdx(i);
                params = obj.form.getParam(i);
                obj.controllers{i} = DiffDriveFollower(type,params);
            end
            % assign actuators
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
            % get current poses of all robots
            poses = obj.world.get_poses();
            controls = cell(1,obj.numRobots);
            % control the leader
            ctl = obj.controllers{1};
            pose = poses(:,1);
            controls{1} = ctl.compute_control(pose,readings);
            % control the followers
            for i = 2:obj.numRobots
                ctl = obj.controllers{i};
                %lead = poses(:,i-1);
                leadIdx = obj.form.getIdx(i);
                lead = poses(:,leadIdx);
                pose = poses(:,i);
                controls{i} = ctl.compute_control(pose,lead);
            end
        end
    end
end

