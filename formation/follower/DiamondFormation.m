classdef DiamondFormation
    %DIAMONDFORMATION 
    % A four-robot formation in diamond shape
    %      1
    %    /   \
    %   2     3
    %    \   /
    %      4
    
    properties
        numRobots
        leaderIdx
        followInfo
    end
    
    methods
        function obj = DiamondFormation()
            %construct a DIAMONDFORMATION
            obj.numRobots = 4;
            leadIdx = cell(3,1);
            leadIdx{1} = 1;
            leadIdx{2} = 1;
            leadIdx{3} = [2;3];
            obj.leaderIdx = leadIdx;
            followInfo = cell(3,1);
            param1.type = "dphi";
            param1.d = 1;
            param1.phi = -pi/6;
            param2.type = "dphi";
            param2.d = 1;
            param2.phi = pi/6;
            param3.type = "dd";
            param3.d1 = 1;
            param3.d2 = 1;
            followInfo{1} = param1;
            followInfo{2} = param2;
            followInfo{3} = param3;
            obj.followInfo = followInfo;
        end
        
        function type = getType(obj,n)
            % return the type of a follower
            fi = obj.followInfo{n-1};
            type = fi.type;
        end
        
        function idx = getIdx(obj,n)
            % get the index of the robot
            % which number nth robot are following
            idx = obj.leaderIdx{n-1};
        end
        
        function params = getParam(obj,n)
            params = obj.followInfo{n-1};
        end
    end
end

