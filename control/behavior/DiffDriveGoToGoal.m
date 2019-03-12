classdef DiffDriveGoToGoal < control
    %GOTOGOAL go-to-goal behavior for behavior-based controller
    % applied to differencitla drive dynamics
    
    properties
        controller
        behavior
    end
    
    methods
        function obj = DiffDriveGoToGoal(RobotInfo,PursuitInfo,goal)
            %GOTOGOAL given a goal location with size (1 2), 
            % construct a controller pointing robot to that goal
            valid_dynamics = ["DiffDrive"];
            if (~ismember(RobotInfo.type,valid_dynamics))
                msg = "Go-to-goal controller: wrong dynamics type";
                error(msg);
            end
            obj.behavior = "go-to-goal";
            % Pure pursuit controller for goal point
            obj.controller = robotics.PurePursuit;
            obj.controller.Waypoints = [goal];
            obj.controller.LookaheadDistance = PursuitInfo.LookaheadDistance;
            obj.controller.DesiredLinearVelocity = PursuitInfo.DesiredLinearVelocity;
            obj.controller.MaxAngularVelocity = PursuitInfo.MaxAngularVelocity;
        end
        
        function control = compute_control(obj,pose,readings)
            %Compute control for go to goal behavior
            [vRef,wRef] = obj.controller(pose);
            control.vRef = vRef;
            control.wRef = wRef;
        end
    end
end

