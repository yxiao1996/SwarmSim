classdef DiffDriveFollower
    %DIFFDRIVEFOLLOWER 
    % controller for followers in the leader-follower method
    
    properties
        type % [d,d] or [d,phi] formation
        d1
        d2
        phi
    end
    
    methods
        function obj = DiffDriveFollower(type,params)
            %DIFFDRIVEFOLLOWER 
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
            if (abs(d_phi) < phi_thresh) % ok to match d
                control.wRef = 0;
                d = sqrt(d_x^2 + d_y^2);
                if (d > obj.d1-delta)&&(d < obj.d1+delta)
                    control.vRef = 0;
                elseif (d >= obj.d1+delta)
                    control.vRef = v;
                else
                    control.vRef = -v;
                end
            else % match phi to leader first
                control.vRef = 0;
                if (d_phi > 0)
                    control.wRef = w;
                else
                    control.wRef = -w;
                end
            end
        end
    end
end

