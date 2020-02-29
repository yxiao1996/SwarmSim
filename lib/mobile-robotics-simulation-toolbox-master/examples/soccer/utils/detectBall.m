function [ballRange,ballAngle] = detectBall(detections)
% Accepts a set of object detections and searches for a ball
%
% Copyright 2019 The MathWorks, Inc.

ballRange = -1;
ballAngle =  0;

for idx = 1:size(detections,1)
    if detections(idx,3) == ObjectType.Ball
        ballRange = detections(idx,1);
        ballAngle = detections(idx,2);
    end
end

end

