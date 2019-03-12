classdef (Abstract)control
    %CONTROLLER Abstract class of robot controllers
    
    properties
        control_bnd
    end
    
    methods (Abstract)
        compute_control(obj,pose,readings)
    end
end

