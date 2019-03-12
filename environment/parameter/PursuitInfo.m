classdef PursuitInfo
    %PURSUITINFO parameters for PurePursuit controllers
    
    properties
        LookaheadDistance
        DesiredLinearVelocity
        MaxAngularVelocity
    end
    
    methods
        function obj = PursuitInfo(lookahead,v_opt,w_max)
            %PURSUITINFO construct a pursuit info object
            obj.LookaheadDistance = lookahead;
            obj.DesiredLinearVelocity = v_opt;
            obj.MaxAngularVelocity = w_max;
        end
       
    end
end

