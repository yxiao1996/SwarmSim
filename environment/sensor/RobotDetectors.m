classdef RobotDetectors
    %ROBOTDETECTORS 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        swarmInfo % information about other robot in the swarm
        idx       % who i am in the swarm
        myInfo    % my information
        es        % sensor vectors for computing sensor masks
        angles    % sensor angles for me
    end
    
    methods
        function obj = RobotDetectors(swarmInfo,idx)
            %ROBOTDETECTORS 
            obj.swarmInfo = swarmInfo;
            obj.idx = idx;
            obj.myInfo = swarmInfo.infos{idx};
            numSensors = obj.myInfo.numSensors;
            obj.angles = linspace(-pi,pi,numSensors);
            sensorRange = obj.myInfo.sensorRange;
            obj.es = zeros(numSensors-1,2);
            %i = [1,0]; j = [0,1]; % unit vectors
            for i = 1:numSensors-1
                theta = obj.angles(i);
                obj.es(i,:) = [cos(theta),sin(theta)]*sensorRange;
            end
        end
        
        function mask = sensor_mask(obj,detections)
            % compute a sensor mask to filter out sensor readings
            % caused by other robot in the swarm
            % this function is used in swarm SLAM 
            numDetections = size(detections,1);
            numSensors = obj.myInfo.numSensors;
            mask = ones(numSensors,1); 
            for i = 1:numSensors
                %e = squeeze(obj.es(i,:));
                theta = obj.angles(i);
                blocked = false; 
                for j = 1:numDetections
                    id = uint8(detections(j,3));
                    r = obj.swarmInfo.infos{id}.body_width/2;
                    phi = detections(j,2);
                    range = detections(j,1);
                    psi = asin(r/range);
                    delta = pi/12;
                    if (theta>(phi-psi-delta) && theta<(phi+psi+delta))
                        blocked = true;
                    end
                end
                if(blocked == true)
                    mask(i) = 0;
                end
            end
            %disp(mask);
        end
    end
end

