classdef (Abstract)obstacle
    %ENTITY base class for all obstacles in the simulated environment
    % the properties stored for each entity are used for visualization
    % 
    
    properties
        position
        orientation
        bound_box   % bounding box apex [left top right bottom]
        type
    end
    
    methods (Abstract)
        InBoundBox(xy); % whether point in bounding box
        InObstacle(xy); % whether point in obstacle
    end
end

