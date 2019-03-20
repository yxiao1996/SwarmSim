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
            phi = angle(direc(1)+1j*direc(2));
            if (phi>pi/2)||(phi<-pi/2)
                if(phi<0)    
                    theta = angle(direc(2)+1j*direc(1));
                else
                    theta = angle(direc(2)-1j*direc(1));
                end
            else
                theta = 0;
            end
            if(theta>0 && theta < 5*pi/6)
                theta = theta + pi/6;
            end
            if(theta<0 && theta > -5*pi/6)
                theta = theta - pi/6;
            end
            k = 0.8;
            wRef = theta*k;
            control.vRef = 0.5;
            control.wRef = wRef;
        end
    end
end

