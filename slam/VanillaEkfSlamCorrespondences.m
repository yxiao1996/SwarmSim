classdef VanillaEkfSlamCorrespondences
    %VANILLAEKFSLAMCORRESPONDENCES 
    % an vanilla implementation of extended Kalman filter SLAM with data
    % association assumption
    
    properties
        mu
        Sigma
        R
        Q
        N
        seen  % mark if a landmark have been seen before
    end
    
    methods
        function obj = VanillaEkfSlamCorrespondences(x,Sigma_r,numLandmarks)
            %VANILLAEKFSLAMCORRESPONDENCES 
            obj.N = numLandmarks;
            obj.mu = zeros(3+3*obj.N,1); % initialize empty mu
            obj.mu(1:3) = x;         % initial position of robot
            obj.Sigma = diag([ones(1,3+3*obj.N)*1000]);
            obj.Sigma(1:3,1:3) = Sigma_r; % initialize robot position covariance
            obj.R = diag([0.01 0.01 0.01]);%zeros(3);
            obj.Q = diag([0.1 0.1 0.0001]);
            obj.seen = zeros(1,obj.N);
        end
        
        function [obj,mu,Sigma] = step(obj,ut,landmark)
            % step the extended Kalman filter
            
            % The Predictoin Step
            Fx = [eye(3) zeros(3,obj.N*3)];
            v = ut(1); w = ut(2);
            dt = 0.05;
            theta = obj.mu(3);
            mu_bar = obj.mu + ...
                Fx'*[
                  cos(theta)*v*dt;
                  sin(theta)*v*dt;
                  w*dt
                ];
            Gt = eye(3) + ...
            Fx'*[
              0 0 -v*sin(theta)*dt;
              0 0 v*cos(theta)*dt;
              0 0 0
            ]*Fx;
            Sigma_bar = Gt*obj.Sigma*Gt' + Fx'*obj.R*Fx;
            ct = landmark(:,3);
            % The Update Step
            for i = 1:obj.N
                j = ct(i);
                rt_i = landmark(i,1);
                phit_i = landmark(i,2);
                st_i = landmark(i,3);
                zt_i = [rt_i;phit_i;st_i];
                % if the landmark never seen before
                if(obj.seen(st_i)==0)
                    obj.seen(st_i)=1; % set seen flag to one
                    muj_bar = [mu_bar(1);mu_bar(2);st_i] + ...
                        rt_i*[cos(phit_i+mu_bar(3));sin(phit_i+mu_bar(3));0];
                    mu_bar(3*j+1:3*j+4) = muj_bar;
                else
                    muj_bar = mu_bar(3*j+1:3*j+4);
                end
                delta_x = muj_bar(1) - mu_bar(1);
                delta_y = muj_bar(2) - mu_bar(2);
                delta = [delta_x;delta_y];
                q = delta'*delta;
                zt_hat_i = [sqrt(q);atan2(delta_y,delta_x)-mu_bar(3);muj_bar(3)];
                Fxj = zeros(6,3+obj.N*3);
                Fxj(1:3,1:3) = eye(3);
                Fxj(4:6,3*j+1:3*j+4) = eye(3);
                Ht_i = 1/q * [
                  sqrt(q)*delta_x -sqrt(q)*delta_y 0  -sqrt(q)*delta_x sqrt(q)*delta_y 0;
                  delta_y         delta_x          -1 -delta_y         -delta_x        0;
                  0               0                0   0               0               1
                ] * Fxj;
                Kt_i = Sigma_bar*Ht_i'/(Ht_i*Sigma_bar*Ht_i'+obj.Q);
                mu = mu + Kt_i*(zt_i-zt_hat_i);
                Sigma = Sigma - Kt_i*Ht_i*Sigma_bar;
            end
            obj.mu = mu;
            obj.Sigma = Sigma;
        end
    end
end

