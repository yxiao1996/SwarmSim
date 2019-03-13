classdef OmniDirFollower < control
    %OMNIDIRFOLLOWER 
    % a controller for followers in leader-follower
    % formation control
    % omni-directional dynamics
    
    properties
        type % [d,d] or [d,phi] formation
        d1
        d2
        phi
    end
    
    methods
        function obj = OmniDirFollower(type,params)
            %OMNIDIRFOLLOWER 
            valid_type = ["dd";"dphi"];
            if (~ismember(type,valid_type))
                msg = "follwer controller: invalid formation type";
                error(msg);
            end
            obj.type = type;
            if (strcmp(type,"dd")) % distance-distnce formation
                obj.d1 = params.d1;
                obj.d2 = params.d2;
            elseif (strcmp(type,"dphi"))
                obj.d1 = params.d;
                obj.phi = params.phi;
            end
        end
        
        function control = compute_control(obj,pose,lead)
            % compute the control according to current position
            if (strcmp(obj.type,"dphi"))
                control = obj.compute_dphi(pose,lead);
            end
        end
        
        function control = compute_dphi(obj,pose,lead)
            % giving the pose of leader compute control
            phi_thresh = 0.1;
            delta = 0.1;
            w = 1.5;
            v = 1.0;
            theta = pose(3);
            x = pose(1); y = pose(2);
            x_l = lead(1); y_l = lead(2);
            d_x = x_l - x;
            d_y = y_l - y;
            theta_l = angle(d_x + 1j*d_y);
            d_phi = theta_l - (theta+obj.phi);
            
        end
    end
end

