function velB = worldToBody(velW,pose)
% WORLD TO BODY Converts a 2D velocity input velW = [vx;vy;w] from 
% world (global) coordinates to body (vehicle) coordinates.
%
% vx = Linear velocity in x-direction (longitudinal)
% vy = Linear velocity in y-direction (lateral)
% w = Angular velocity (about z-axis)
%
% Copyright 2018 The MathWorks, Inc.

    theta = pose(3);
    velB = [cos(theta) sin(theta) 0;-sin(theta) cos(theta) 0;0 0 1]*velW;

end
