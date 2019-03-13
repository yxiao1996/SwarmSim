classdef DiffDrivePursueWayPoints < control
    %DIFFDRIVEPURSUEWAYPOINTS 
    % pursue way points control for differential drive dynamics
    % using purepursuit controller from robotics system toolbox
    properties
        controller
    end
    
    methods
        function obj = DiffDrivePursueWayPoints(waypoints)
            obj.controller = robotics.PurePursuit;
            obj.controller.Waypoints = waypoints;
            obj.controller.LookaheadDistance = 0.35;
            obj.controller.DesiredLinearVelocity = 0.6;
            obj.controller.MaxAngularVelocity = 1.0;
        end
        
        function control = compute_control(obj,pose,readings)
            [vRef,wRef] = obj.controller(pose);
            control.vRef = vRef;
            control.wRef = wRef;
        end
    end
end

