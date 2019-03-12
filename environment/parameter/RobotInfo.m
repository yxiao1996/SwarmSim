classdef RobotInfo
    %ROBOTINFO parameter of robot
    %   此处显示详细说明
    
    properties
        type         % type of dynamics
        wheel_radius % radius of wheels
        body_width   % width of chassis
        numSensors   % number of range finders mounted
        sensorRange  % max range of each range finder
    end
    
    methods
        function obj = RobotInfo(t,R,L,s,r)
            %ROBOTINFO construct a robotInfo object
            % check dynamics type
            valid_dynamics = ["DiffDrive";"OmniDir"];
            if (~ismember(t,valid_dynamics))
                msg = "Robot Information: wrong dynamics type";
                error(msg);
            end
            obj.type = t;
            obj.wheel_radius = R;
            obj.body_width = L;
            obj.numSensors = s;
            obj.sensorRange = r;
        end
        
        function type = get_type(obj)
            % return the type of dynamics
           type = string(obj.type);
        end
    end
end

