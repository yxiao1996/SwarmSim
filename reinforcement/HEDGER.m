classdef HEDGER
    %HEDGER 
    % HEDGER function approximator for Q-learning in continuous space
    
    properties
        Q % [numPoints,dim(q)+1]: an array on Q values used for query
        default % default q value
        k_thresh
        k_min
        maxNumPoints
        curNumPoints
    end
    
    methods
        function obj = HEDGER(Q,default,k_thresh,k_min)
            obj.maxNumPoints = 200;
            obj.curNumPoints = size(Q,1);
            obj.Q = zeros(obj.curNumPoints,size(Q,2)); % initialize the HEDGER
            for i = 1:obj.curNumPoints
                obj.Q(i,:) = Q(i,:);
            end
            obj.default = default;
            obj.k_thresh = k_thresh;
            obj.k_min = k_min;
        end
        
        function obj = training(obj,s,a,r,s_,alpha,gamma,h)
            % train HEDGER for one step
            [q,K,w] = obj.predict(s,a,h);
            [s_gred,a_gred] = obj.greedy(s_,a,h,0.1,0.1);
            
            q_next = obj.predict(s_gred,a_gred,h);
            q_new = q + alpha*(r+gamma*q_next-q);
            idx = mod(obj.curNumPoints,obj.maxNumPoints);
            obj.Q(idx,:) = [s a q_new];
            numKs = size(K,1);
            q_idx = size(K,2);
            for i = 1:numKs
                k = squeeze(K(i,:));
                w_i = w(i);
                idx = find(obj.Q==k);
                prev_q = obj.Q(idx(1),q_idx);
                obj.Q(idx,q_idx) = prev_q + w_i*(q_new-prev_q);
            end
        end
        
        function [s_gred,a_gred] = greedy(obj,s,a,h,d,thresh)
            % generate sample around s and a with range d
            numSamples = 10;
            s_rand = d*(rand(numSamples,length(s))-0.5) + s;
            a_rand = d*(rand(numSamples,length(a))-0.5) + a;
            samples = zeros(numSamples,1);
            % initialize
            for i = 1:numSamples
                pred = obj.predict(s,a_rand(i,:),h);
                samples(i) = pred;
            end
            % find max of sample
            prev_max = max(samples);
            while (true)
                s_rand = d*(rand(numSamples,length(s))-0.5) + s;
                a_rand = d*(rand(numSamples,length(a))-0.5) + a;
                for i = 1:numSamples
                    pred = obj.predict(s,a_rand(i,:),h);
                    samples(i) = pred;
                end
                curr_max = max(samples);
                if(abs(prev_max-curr_max)<thresh)
                    max_idx = find(samples==curr_max);
                    s_gred = s; 
                    a_gred = a_rand(max_idx(1),:);
                    break
                else
                    prev_max = curr_max;
                end
            end
        end
        
        function [Qpred,K,weights] = predict(obj,s,a,h)
            %HEDGER prediction
            % s: [1,dim(state)] state vector
            % a: [1,dim(action)] action vector
            q = [s a]; % concatenate s and a to form q
            dim_q = length(q);
            K = obj.findKNeighbors(q);
            numKs = size(K,1);
            %disp(numKs);
            weights = zeros(numKs,1);
            if(numKs < obj.k_min)
                Qpred = obj.default;
                weights = ones(numKs,1)/numKs;
            else
                % calculate the convex hull of K
                K_q = K(:,1:dim_q); % design matrix
                %disp(size(K_q));
                ch_K = convhulln(K_q);
                ch_Kq = convhulln([K_q;q]);
                if(isequal(ch_K,ch_Kq)) % two convex hulls are equal
                    % calculate weights
                    for i = 1:numKs
                        k_i = squeeze(K_q(i,:));
                        w_i = exp(-dot(q-k_i,q-k_i)/(h^2));
                        weights(i) = w_i;
                    end
                    q_t = zeros(numKs,1); % target values
                    for i = 1:numKs
                        q_t(i) = K(i,dim_q+1);
                    end
                    W = diag(weights);
                    w_ml = obj.WeightedLeastSquare(K_q,q_t,W);
                    Qpred = w_ml' * q';
                else
                    Qpred = obj.default;
                    weights = ones(numKs,1)/numKs;
                end
            end
        end
        
        function w_ml = WeightedLeastSquare(obj,Phi,t,W)
            w_ml = (Phi'*W*Phi)\(Phi'*W*t);
        end
        
        function K = findKNeighbors(obj,q)
            K = []; % K: [k,dim(q)]
            dists = zeros(size(obj.Q,1),1);
            dim_q = length(q);
            for i = 1:size(obj.Q,1)
                datum = squeeze(obj.Q(i,:));
                q_train = datum(1:dim_q);
                dists(i) = obj.EuclidDistance(q,q_train);
            end
            max_val = max(dists);
            for i = 1:size(obj.Q,1)
                min_val = min(dists);
                if(min_val < obj.k_thresh)
                    min_idx = find(dists==min_val);
                    K = [K;obj.Q(min_idx(1),:)];
                    dists(min_idx) = max_val;
                else
                    break;
                end
            end
        end
        
        function d = EuclidDistance(obj,x1,x2)
            vec = x2(:) - x1(:);
            d = sqrt(dot(vec,vec));
        end
    end
end

