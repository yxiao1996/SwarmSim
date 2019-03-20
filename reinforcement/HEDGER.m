classdef HEDGER
    %HEDGER 
    % HEDGER function approximator for Q-learning in continuous space
    
    properties
        Q % [numPoints,dim(q)+1]: an array on Q values used for query
        default % default q value
        k_thresh
        k_min
    end
    
    methods
        function obj = HEDGER(default,k_thresh,k_min)
            obj.default = default;
            obj.k_thresh = k_thresh;
            obj.k_min = k_min;
        end
        
        function training(obj,s,a,r,s_,alpha,gamma,h)
            % train HEDGER for one step
            [q,K,w] = obj.predict(s,a,h);
            
        end
        
        function [Qpred,K,weights] = predict(obj,s,a,h)
            %HEDGER prediction
            % s: [1,dim(state)] state vector
            % a: [1,dim(action)] action vector
            q = [s a]; % concatenate s and a to form q
            dim_q = length(q);
            K = obj.findKHeighbors(q);
            numKs = size(K,1);
            weights = zeros(numKs,1);
            if(numKs < obj.k_min)
                Qpred = obj.default;
                weights = ones(numKs,1)/numKs;
            else
                % calculate the convex hull of K
                K_q = K(:,1:dim_q); % design matrix
                ch_K = convhulln(K_q);
                ch_Kq = convhulln([K_q;q]);
                if(isequal(ch_K,ch_Kq) % two convex hulls are equal
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
                    w_ml = WeightedLeastSquare(K_q,q_t,W);
                    Qpred = w_ml' * q;
                else
                    Qpred = obj.default;
                    weights = ones(numKs,1)/numKs;
                end
            end
        end
        
        function w_ml = WeightedLeastSquare(obj,Phi,t,W)
            w_ml = (Phi'*W*phi)\(Phi'*W*t);
        end
        
        function K = findKNeighbors(obj,q)
            K = []; % K: [k,dim(q)]
            dists = zeros(size(obj.Q,1),1);
            dim_q = length(q);
            for i = 1:size(obj.Q,1)
                datum = squeeze(bj.Q(i,:));
                q_train = datum(1:dim_q);
                dists(i) = obj.EuclidDistance(q,q_train);
            end
            max_val = max(dists);
            for i = 1:size(obj.Q,1)
                min_val = min(dists);
                if(min_val < obj.k_thresh)
                    min_idx = find(dists==min_val);
                    K = [K;obj.Q(min_idx,:)];
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

