classdef DiffDriveBehaviorBased < control
    %DIFFDRIVEBEHAVIORBASED 
    % behavior based controller for robot to navigate 
    % differential drive dynamics
    % including the following behaviors:
    % Go to Goal
    
    properties
        fsm
        goal
        behaviors
        numBehaviors
        controllers
        sensors
    end
    
    methods
        function obj = DiffDriveBehaviorBased(robotInfo,goal)
            %DIFFDRIVEBEHAVIORBASED 
            % contruct a behavior-based controller
            valid_dynamics = ["DiffDrive"];
            if (~ismember(robotInfo.type,valid_dynamics))
                msg = "Behavior-based controller: wrong dynamics type";
                error(msg);
            end
            obj.goal = goal;
            obj.behaviors = {'go-to-goal';'avoid-wall'};
            obj.numBehaviors = size(obj.behaviors,1);
            obj.controllers = cell(1,obj.numBehaviors);
            obj.sensors = RangeFinders(robotInfo);
            pursuitInfo = PursuitInfo(0.35,0.75,1.5);
            for i = 1:obj.numBehaviors
                if (obj.behaviors(i,:) == "go-to-goal")
                    obj.controllers{i} = DiffDriveGoToGoal(robotInfo,pursuitInfo,goal);
                end
                if (obj.behaviors(i,:) == "avoid-wall")
                    obj.controllers{i} = DiffDriveAvoidWall(robotInfo,pursuitInfo);
                end
            end
            % initialize finite state machine
            obj.fsm = BehaviorBasedFSM("go-to-goal");
        end
        
        function control = compute_control(obj,pose,raw_reads)
            % collect readings from sensors
            reads = obj.sensors.fill_nan(raw_reads);
            direc_avoid = obj.sensors.avoid_wall(reads);
            % check for state switching
            angle_g = pose(3);
            R = [
              cos(angle_g) -sin(angle_g);
              sin(angle_g)  cos(angle_g);
            ];
            vec_g = obj.goal - pose(1:2)';
            direc_goal = R'*vec_g';
            [state,obj.fsm] = obj.fsm.compute_next_state(reads,direc_avoid,direc_goal);
            disp(state);
            %compute control by according to state
            switch (state)
                case "go-to-goal" % Go-To-Goal State
                    controller_idx = find(strcmp(obj.behaviors, state)==1);
                    controller = obj.controllers{controller_idx};
                    control = controller.compute_control(pose,reads);
                case "avoid-wall"
                    controller_idx = find(strcmp(obj.behaviors, state)==1);
                    controller = obj.controllers{controller_idx};
                    control = controller.compute_control(pose,direc_avoid);
                otherwise
                    control.vRef = 0;
                    control.wRef = 0;
            end
        end
    end
end

