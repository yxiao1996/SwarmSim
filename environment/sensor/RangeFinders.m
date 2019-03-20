classdef RangeFinders
    %RANGEFINDERS range finder sensor post processing
    % use max range to fill out nan readings
    % detect walls using range finder
    properties
        numSensors
        max_range
        angles   % angles
        es       % unit vector to perform avoid wall
    end
    
    methods
        function obj = RangeFinders(robotInfo)
            %RANGEFINDERS 
            obj.numSensors = robotInfo.numSensors;
            obj.max_range = robotInfo.sensorRange;
            obj.angles = linspace(-pi,pi,robotInfo.numSensors);
            obj.es = zeros(obj.numSensors-1,2);
            %i = [1,0]; j = [0,1]; % unit vectors
            for i = 1:obj.numSensors-1
                theta = obj.angles(i);
                obj.es(i,:) = [cos(theta),sin(theta)];
            end
        end
        
        function direction = avoid_wall(obj,reads)
            % compute the direction to avoid wall
            % reads = obj.fill_nan(raw_reads);
            sensor_sum = [0 0];
            reads = obj.discard_back(reads);
            for i = 1:obj.numSensors-1
                read = reads(i);
                sensor_sum = sensor_sum + obj.es(i,:).*read;
            end
            direction = sensor_sum ./ (obj.numSensors-1);
        end
        
        function new_reads = discard_back(obj,reads)
            new_reads = zeros(size(reads));
            s1 = floor(obj.numSensors/4);
            s2 = ceil(obj.numSensors*3/4);
            for i = 1:obj.numSensors 
                if (i <=s1 || i >=s2)
                    new_reads(i) = obj.max_range;
                else
                    new_reads(i) = reads(i);
                end
            end
        end
        
        function filled_reads = fill_nan(obj,raw_reads)
            % fill nans in raw readings with maximum sensor range
            %disp(raw_reads);
            filled_reads = fillmissing(raw_reads,'constant',obj.max_range);
        end
    end
end

