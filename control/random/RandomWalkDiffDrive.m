classdef RandomWalkDiffDrive < control
    % RANDOM WALK CONTROLLER for a single robot
    
    properties
        v_rng
        w_rng
        v_ofs
        w_ofs
    end
    
    methods
        function obj = RandomWalkDiffDrive(v_bnd,w_bnd)
            obj.v_rng = v_bnd(2) - v_bnd(1);
            obj.v_ofs = v_bnd(1);
            obj.w_rng = w_bnd(2) - w_bnd(1);
            obj.w_ofs = w_bnd(1);
        end
        
        function control = compute_control(obj,pose,readings)
            control.vRef = rand*obj.v_rng + obj.v_ofs;
            control.wRef = rand*obj.w_rng + obj.w_ofs;
        end
    end
end

