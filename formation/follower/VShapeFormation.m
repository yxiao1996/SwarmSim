classdef VShapeFormation < LFformation
    %VSHAPEFORMATION 
    % A five-robot formation in V shape
    %           1
    %         /   \
    %        2     3
    %       /       \
    %      4         5
    
    properties
    end
    
    methods
        function obj = VShapeFormation()
            %VSHAPEFORMATION 
            obj.numRobots = 5;
            leadIdx = cell(4,1);
            leadIdx{1} = 1;
            leadIdx{2} = 1;
            leadIdx{3} = 2;
            leadIdx{4} = 3;
            obj.leaderIdx = leadIdx;
            followInfo = cell(4,1);
            param1.type = "dphi";
            param1.d = 0.7;
            param1.phi = -pi/6;
            param2.type = "dphi";
            param2.d = 0.7;
            param2.phi = pi/6;
            param3.type = "dphi";
            param3.d = 0.7;
            param3.phi = -pi/6;
            param4.type = "dphi";
            param4.d = 0.7;
            param4.phi = pi/6;
            followInfo{1} = param1;
            followInfo{2} = param2;
            followInfo{3} = param3;
            followInfo{4} = param4;
            obj.followInfo = followInfo;
        end
        
    end
end

