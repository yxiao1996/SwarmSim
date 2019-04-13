classdef EKFLocalizationSimulation < simulation
    %BEHAVIORBASEDSIMULATION 
    % A simulation for robots navigation using behavior-based method
    
    properties
        localizers
    end
    
    methods
        function obj = EKFLocalizationSimulation(map,swarmInfo,numLandmarks)
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
                obj.controllers{i} = DiffDriveBehaviorBasedBlend(robotInfo,goal);
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
            % add random landmarkd in the world
            landmarks_xy = rand(numLandmarks,2)*10 + 2.5;
            labels = [1:numLandmarks]';
            data = [landmarks_xy labels];
            obj.world = obj.world.addLandmarks(data);
            % associate each robot with one localizer
            obj.localizers = cell(1,obj.numRobots);
            for i = 1:obj.numRobots
                localizer = EKFLocalizerCorrespondences(obj.world.landmarks,...
                    swarmInfo.poses(:,i),...
                    eye(3)*0.5);
                obj.localizers{i} = localizer;
            end
        end
        
        function obj = step(obj)
            readings = obj.sensor_phase();
            controls = obj.control_phase(readings);
            
            poses = obj.actuate_phase_(controls);
            obj = obj.physics_phase(poses);
            obj = obj.localize_phase(controls);
            obj.visualize_();
            
        end
        
        function obj = localize_phase(obj,controls)
            landmarks = obj.world.readCameras();
            hold on
            delete(obj.hPlot);
            hPlot_ = zeros(obj.numRobots,1);
            poses = obj.world.get_poses();
            for i = 1:obj.numRobots
                control = controls{i};
                ut = [control.vRef;control.wRef];
                landmark = landmarks{i};
                %disp(size(landmark));
                localizer = obj.localizers{i};
                pose = poses(:,i);
                [localizer,mu,Sigma] = localizer.step(ut,landmark,pose);
                disp(Sigma);
                obj.localizers{i} = localizer;
                hPlot_(i) = obj.draw_ellipse(mu,Sigma);
            end
            obj.hPlot = hPlot_;
            hold off
        end
        
        function h = draw_ellipse(obj,mu,Sigma)
            mu_ = mu(1:2);
            Sigma_ = Sigma(1:2,1:2);
            h = error_ellipse(Sigma_,mu_);
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

