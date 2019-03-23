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
                    if (r>range)
                        blocked = true;
                        break;
                    end
                    psi = asin(r/range);
                    delta = pi/24;
                    %b = obj.checkBetween(theta,phi,psi); 
                    %if (b==true)
                    if(theta>=(phi-psi-delta) && theta<=(phi+psi+delta))
                        blocked = true;
                        break;
                    end
                end
                if(blocked == true)
                    mask(i) = 0;
                    if(i>1)
                        mask(i-1) = 0;
                    end
                    if(i<numSensors)
                        mask(i+1) = 0;
                    end
                end
            end
            %disp(mask);
        end
        
        function b = checkBetween(obj,a,a1,a2)
            % check if a is between a1 and a2
            % construct complex number on unit circle
            v_a = cos(a)+1j*sin(a);
            v_a1 = cos(a1)+1j*sin(a1);
            v_a2 = cos(a2)+1j*sin(a2);
            d1 = angle(v_a/(v_a1*v_a2));
            d2 = angle(v_a/(v_a1/v_a2));
            disp(d1*d2);
            if (d1*d2<0)
                b = true;
            else
                b = false;
            end
        end
        
    end
end

