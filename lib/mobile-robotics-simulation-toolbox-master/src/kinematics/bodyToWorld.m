function velW = bodyToWorld(velB,pose)
% BODYTOWORLD Converts a 2D velocity input velB = [vx;vy;w] from body 
% (vehicle) coordinates to world (global) coordinates.
%
% vx = Linear velocity in x-direction (longitudinal)
% vy = Linear velocity in y-direction (lateral)
% w = Angular velocity (about z-axis)
%
% Copyright 2018 The MathWorks, Inc.

    theta = pose(3);
    velW = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1]*velB;

end
