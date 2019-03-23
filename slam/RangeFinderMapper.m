classdef RangeFinderMapper
    %RANGEFINDERMAPPER 
    % use readings of range sensors and robot detectors
    % to recover the map
    
    properties
        points
        n
        count
        angles
        es
    end
    
    methods
        function obj = RangeFinderMapper(numPoints,swarmInfo)
            %RANGEFINDERMAPPER 
            obj.points = zeros(numPoints,2);
            obj.n = numPoints;
            obj.count = 0;
            numSensors = swarmInfo.infos{1}.numSensors;
            obj.angles = linspace(-pi,pi,numSensors);
            sensorRange = swarmInfo.infos{1}.sensorRange;
            obj.es = zeros(numSensors-1,2);
            %i = [1,0]; j = [0,1]; % unit vectors
            for i = 1:numSensors-1
                theta = obj.angles(i);
                obj.es(i,:) = [cos(theta),sin(theta)];
            end
            
        end
        
        function obj = addPoints(obj,pose,readings,mask)
            % insert new points into the map
            % based on range finder readings and mask computed by robot
            % detectors
            numReads = size(readings,1); 
            theta = pose(3);
            t = pose(1:2);
            R = [
              cos(theta) -sin(theta);
              sin(theta) cos(theta)
            ];
            
            for i = ceil(numReads/4):ceil(numReads*3/4)
                read = readings(i);
                if (~isnan(read) && mask(i)~=0)
                    e = squeeze(obj.es(i,:))*read;
                    new_point = R*e' + t;
                    new_idx = mod(obj.count,obj.n)+1;
                    obj.points(new_idx,:) = new_point;
                    obj.count = new_idx;
                end
            end
        end
    end
end

