function map = createMapFromName(mapName)
% CREATEMAPFROMNAME Outputs a robotics.BinaryOccupancyGrid object given the
% name of the map variable. This is necessary because MATLAB objects are
% currently not supported as Simulink mask parameters
%
% Copyright 2018 The MathWorks, Inc.

    % Default value is empty
    map = [];

    % First, check whether the map name is empty
    if isempty(mapName)
        return;
    end

    % Next, check the base and current model workspaces for a valid variable
    % with the specified name
    try
        map = evalin('base',mapName);
    catch
        try
            mdlWs = get_param(bdroot,'ModelWorkspace');
            map = getVariable(mdlWs,mapName);
        catch
            error(['Invalid map name ''' num2str(mapName) ''' in base and model workspaces.']);
        end
    end

    % Check the data type of the map
    % If it is a robotics.OccupancyGrid, return it as is
    if isa(map,'robotics.OccupancyGrid')
        return;
        % If it is a robotics.BinaryOccupancyGrid, convert it to a
        % robotics.OccupancyGrid. This provides access to necessary methods such as
        % RAYCAST and RAYINTERSECTION
    elseif isa(map,'robotics.BinaryOccupancyGrid')
        map = binToOccGrid(map);
    else
        error(['Map name ''' num2str(mapName) ''' must be a robotics.OccupancyGrid or robotics.BinaryOccupancyGrid object.']);        
    end

end

function occGrid = binToOccGrid(binGrid)
% BINTOOCCGRID Converts a robotics.BinaryOccupancyGrid object to an
% equivalent robotics.OccupancyGrid object.

    occGrid = binaryOccupancyMap(binGrid.occupancyMatrix,binGrid.Resolution);
    occGrid.GridLocationInWorld = binGrid.GridLocationInWorld;

end

