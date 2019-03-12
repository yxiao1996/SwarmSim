classdef DiffDriveFollowVS
    %DIFFDRIVEFOLLOWVS 
    % A controller for robot embedded in a virtual structure
    
    properties
        lookahead
        opt_v
        max_w
        thresh
    end
    
    methods
        function obj = DiffDriveFollowVS(thresh)
            %DIFFDRIVEFOLLOWVS 
            obj.lookahead = 0.35;
            obj.opt_v = 0.2;
            obj.max_w = 1.5;
            obj.thresh = thresh;
        end
        
        function control = compute_control(obj,targetpoint,pose)
            dist = sqrt((targetpoint(1)-pose(1))^2+(targetpoint(2)-pose(2))^2);
            if (dist < obj.thresh)
                control.vRef = 0;
                control.wRef = 0;
            else
                controller = robotics.PurePursuit;
                controller.Waypoints = [targetpoint];
                controller.LookaheadDistance = obj.lookahead;
                controller.DesiredLinearVelocity = obj.opt_v;
                controller.MaxAngularVelocity = obj.max_w;
                [vRef,wRef] = controller(pose);
                control.vRef = vRef;
                control.wRef = wRef;
            end
        end
    end
end

