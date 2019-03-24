classdef RangeFinderNoisyMapper
    %RANGEFINDERMAPPER 
    % use readings of range sensors and robot detectors
    % to recover the map
    
    properties
        points
        n
        count
        angles
        es
        Rt  % state trainsitoin noise covariance
        Qt  % observation noice covariance
        filters % a collections of kalman filters
        trig
    end
    
    methods
        function obj = RangeFinderNoisyMapper(numPoints,swarmInfo,flt)
            %RANGEFINDERMAPPER 
            obj.points = zeros(numPoints,2);
            obj.n = numPoints;
            obj.count = 0;
            numRobots = swarmInfo.numRobots;
            numSensors = swarmInfo.infos{1}.numSensors;
            obj.angles = linspace(-pi,pi,numSensors);
            sensorRange = swarmInfo.infos{1}.sensorRange;
            obj.es = zeros(numSensors-1,2);
            %i = [1,0]; j = [0,1]; % unit vectors
            for i = 1:numSensors-1
                theta = obj.angles(i);
                obj.es(i,:) = [cos(theta),sin(theta)];
            end
            
            obj.Rt = diag([0.01 0.01 0.01]);
            obj.Qt = diag([0.1 0.1 0.1]);
            obj.filters = cell(1,numRobots);
            for i = 1:numRobots
                init_pose = swarmInfo.poses(:,i);
                kf = KalmanFilterDiffDrive(init_pose);
                obj.filters{i} = kf;
            end
            obj.trig = flt;
        end
        
        function obj = addPoints(obj,pose,readings,mask,control,idx)
            % insert new points into the map
            % based on range finder readings and mask computed by robot
            % detectors
            numReads = size(readings,1); 
            pose = pose + obj.Qt*randn(3,1); % add observation noise
            % update kalman filter
            if(obj.trig)
                u = [control.vRef;control.wRef];
                filt = obj.filters{idx};
                [filt,pose] = filt.step(u,pose);
                obj.filters{idx} = filt;
            end
            % transform the points
            theta = pose(3);
            t = pose(1:2);
            R = [
              cos(theta) -sin(theta);
              sin(theta) cos(theta)
            ];
            % add valid points to map
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

