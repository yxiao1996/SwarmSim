classdef SwarmInfo
    %SWARMINFO parameter of multi-agent system
    
    properties
        numRobots % number of robots in the system
        infos     % parameters for each robot, cell array with type RobotInfo
        poses     % current poses of the system, size (3,numRobots)
        showTraj  % flag if show trajectory
    end
    
    methods
        
        function obj = SwarmInfo(numRobots,infos,poses,show)
            %SWARMINFO construct a swarm info object
            if (numRobots ~= length(infos))
                msg = "numRobots and length of robot information don't match";
                error(msg);
            end
            if (numRobots ~= size(poses,2))
                msg = "pose matrix wrong shape, should have numRobots dimensiona on axis 2";
                error(msg);
            end
            if (size(poses,1) ~= 3)
                msg = "pose have wrong dimension, axis 1 should have 3 dimensions";
                error(msg);
            end
            obj.numRobots = numRobots;
            obj.infos = infos;
            obj.poses = poses;
            obj.showTraj = show;
        end
        
    end
end

