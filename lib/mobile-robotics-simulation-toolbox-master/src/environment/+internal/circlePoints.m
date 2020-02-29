function [xPts,yPts] = circlePoints(x,y,r,numPts)
% CIRCLEPTS Creates points to visualize a circle centered at (x,y), 
% with radius r. The circle contains numPts discrete points.

    anglePts = linspace(-pi,pi,numPts);   
    xPts = x + r*cos(anglePts);
    yPts = y + r*sin(anglePts);

end

