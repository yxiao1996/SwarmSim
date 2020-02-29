function ranges = circleLineIntersection(sensorPose,scanAngles,maxRange,targetPose,targetRadius)
% Finds the range of a lidar sensor intersecting with a circular robot
% Source: http://www.ambrsoft.com/TrigoCalc/Circles2/circlrLine_.htm

numScans = numel(scanAngles);
ranges = nan(numScans,1);

for idx = 1:numScans
    
    scanAngle = scanAngles(idx);
    
    % Line coefficients
    lineAngle = sensorPose(3) + scanAngle;
    m = tan(lineAngle);
    d = sensorPose(2) - m*sensorPose(1);
    
    % Circle coefficients
    a = targetPose(1);
    b = targetPose(2);
    r = targetRadius;
    
    % Find overlap
    del = (r^2 *(1+ m^2)) - (b - m*a - d)^2;
    
    % Find intersection points
    range = NaN;
    
    if del >= 0
        % Solve the quadratic formula for x
        cx = a + m*(b-d);
        c2 = sqrt(del);
        c3 = 1 + m^2;
        x1 = (cx + c2) / c3;
        x2 = (cx - c2) / c3;
        
        % Find y points
        if abs(x1-x2) < 1e-6
            % If x points are close to each other, line is nearly vertical
            % Plug x value into circle equation to get y
            c1 = sqrt(a + r - x1);
            c2 = sqrt(r - a + x1);
            y1 = b + c1*c2;
            y2 = b - c1*c2;
        else
            % Else, plug x values into line equation to get y
            y1 = m*x1 + d;
            y2 = m*x2 + d;
        end
        
        % Find the angles and ranges
        v1 = [x1;y1] - [sensorPose(1);sensorPose(2)];
        v2 = [x2;y2] - [sensorPose(1);sensorPose(2)];
        v = [cos(lineAngle);sin(lineAngle)];
        r1 = norm(v1);
        r2 = norm(v2);
        a1 = v1'*v/r1;
        a2 = v2'*v/r2;
        
        % Find the nearest valid range (based on distance and direction)
        if (r1 <= maxRange) || (r2 <= maxRange)
            if (a1 > 0) && (a2 > 0)
                range = min(r1,r2);
            elseif (a1 > 0)
                range = r1;
            elseif (a2 > 0)
                range = r2;
            end
        end
    end
    
    ranges(idx) = range;
    
end

end

