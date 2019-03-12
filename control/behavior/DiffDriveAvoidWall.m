classdef DiffDriveAvoidWall < control
    %DIFFDRIVEAVOIDWALL 
    % wall-avoiding behavior for bahevior-based controller
    
    properties
        controller
        behavior
    end
    
    methods
        function obj = DiffDriveAvoidWall(robotInfo,PursuitInfo)
            %DIFFDRIVEAVOIDWALL 
            valid_dynamics = ["DiffDrive"];
            if (~ismember(robotInfo.type,valid_dynamics))
                msg = "avoid-wall controller: wrong dynamics type";
                error(msg);
            end
            obj.behavior = "avoid-wall";
            % obj.sensor = RangeFinders(robotInfo);
        end
        
        function control = compute_control(obj,pose,direc)
            %compute control to avoid walls
            theta = angle(direc(2)+1j*direc(1));
            k = 0.5;
            if (theta > 0)
                wRef = theta*k; %0.5;
            else
                wRef = theta*k; %-0.5;
            end
            control.vRef = 0.5;
            control.wRef = wRef;
        end
    end
end

