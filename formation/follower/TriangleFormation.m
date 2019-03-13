classdef TriangleFormation < LFformation
    %TRIANGLEFORMATION 
    % A six-robot formation in triangular shape
    %          1
    %         / \
    %        2 - 3
    %       / \   \
    %      4 - 5 - 6
    
    properties
    end
    
    methods
        function obj = TriangleFormation()
            %TRIANGLEFORMATION 
            obj.numRobots = 6;
            leadIdx = cell(5,1);
            leadIdx{1} = 1;
            leadIdx{2} = [1;2];
            leadIdx{3} = 2;
            leadIdx{4} = [2,4];
            leadIdx{5} = [3;5];
            obj.leaderIdx = leadIdx;
            
            r = 0.7;
            followInfo = cell(5,1);
            param1.type = "dphi";
            param1.d = r; param1.phi = -pi/6;
            param2.type = "dd";
            param2.d1 = r; param2.d2 = r;
            param3.type = "dphi";
            param3.d = r; param3.phi = -pi/6;
            param4.type = "dd";
            param4.d1 = r; param4.d2 = r;
            param5.type = "dd";
            param5.d1 = r; param5.d2 = r;
            followInfo{1} = param1;
            followInfo{2} = param2;
            followInfo{3} = param3;
            followInfo{4} = param4;
            followInfo{5} = param5;
            obj.followInfo = followInfo;
        end
        
    end
end

