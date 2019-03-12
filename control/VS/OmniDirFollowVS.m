classdef OmniDirFollowVS
    %OMNIDIRFOLLOWVS 此处显示有关此类的摘要
    %  A controller for robot embedded in virtual structure
    % omni-directional dynamics
    
    properties
        kv
        kw
        thresh
    end
    
    methods
        function obj = OmniDirFollowVS()
            %OMNIDIRFOLLOWVS 
            obj.thresh = 0.1;
            obj.kv = 1;
            obj.kw = 1;
        end
        
        function control = compute_control(obj,pose_t,pose)
            x_t = pose_t(1); 
            y_t = pose_t(2);
            theta_t = 0;%pose_t(3);
            x = pose(1);
            y = pose(2);
            theta = pose(3);
            dist = sqrt((x_t-x)^2+(y_t-y)^2);
            if (dist < obj.thresh)
                control.xRef = 0;
                control.yRef = 0;
                control.wRef = 0;
            else
                d_x = x_t - x;
                d_y = y_t - y;
                d_theta = theta_t - theta;
                control.xRef = d_x * obj.kv;
                control.yRef = d_y * obj.kv;
                control.wRef = d_theta * obj.kw;
            end
        end
    end
end

