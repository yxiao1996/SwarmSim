classdef LeaderFollowerNoisySimulation < simulation
    %LEADERFOLLOWERSIMULATION 
    % A Leader-Follower formation control simulation
    
    properties
        form
        numFollowers
    end
    
    methods
        function obj = LeaderFollowerNoisySimulation(map,swarmInfo,form)
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
                pose = swarmInfo.poses(:,i);
                obj.controllers{i} = DiffDriveFollowerKalman(type,params,pose);
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
            obj.visualize_();
        end
        
        function obj = step(obj)
            readings = obj.sensor_phase();
            [obj,controls] = obj.control_phase(readings);
            poses = obj.actuate_phase_(controls);
            obj = obj.physics_phase(poses);
            obj.visualize_();
        end
        
        function [obj,controls] = control_phase(obj,readings)
            % get current poses of all robots
            poses = obj.world.get_poses();
            controls = cell(1,obj.numRobots);
            % control the leader
            ctl = obj.controllers{1};
            pose = poses(:,1);
            controls{1} = ctl.compute_control(pose,readings);
            % control the followers
            hPlot_ = zeros(obj.numRobots,1);
            hold on
            %delete(obj.hPlot);
            for i = 2:obj.numRobots
                ctl = obj.controllers{i};
                type = obj.form.getType(i);
                leadIdx = obj.form.getIdx(i);
                pose = poses(:,i);
                pose = AdditiveGaussian(pose',eye(3)*0.1)'; % add gaussian noise
                if (strcmp(type,"dphi"))
                    lead = poses(:,leadIdx);
                    [ctl,controls{i},mu,Sigma] = ctl.compute_control(pose,lead,[0;0;0]);
                elseif (strcmp(type,"dd"))
                    lead1 = poses(:,leadIdx(1));
                    lead2 = poses(:,leadIdx(2));
                    [ctl,controls{i},mu,Sigma] = ctl.compute_control(pose,lead1,lead2);
                end
                obj.controllers{i} = ctl;
                % hPlot_(i) = obj.draw_ellipse(mu,Sigma);
            end
            obj.hPlot = hPlot_;
            hold off
        end
        
        function h = draw_ellipse(obj,mu,Sigma)
            mu_ = mu(1:2);
            Sigma_ = Sigma(1:2,1:2);
            h = error_ellipse(Sigma_,mu_);
        end
        
    end
end

