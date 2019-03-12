classdef (Abstract)actuator
    %ACTUATOR Absctract class of robot actuators
    
    properties
        dynamics
    end
    
    methods (Abstract)
        actuate(obj,controls,pose)
    end
    
    
end

