function waypoints = planPRM(map,pose,goal)
    planner = robotics.PRM(map);
    planner.NumNodes = 200;
    planner.ConnectionDistance = 2;

    % Find a path from the start point to a specified goal point
    startPoint = pose(1:2)';
    goalPoint = goal;
    waypoints = findpath(planner,startPoint,goalPoint);
end

