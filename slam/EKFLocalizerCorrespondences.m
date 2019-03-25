classdef EKFLocalizerCorrespondences
    %EKFLOCALIZERCORRESPONDENCES 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        map
        mu
        Sigma
        R
        Q
    end
    
    methods
        function obj = EKFLocalizerCorrespondences(map,x,Sigma)
            %EKFLOCALIZERCORRESPONDENCES 
            obj.map = map;
            obj.mu = x;
            obj.Sigma = Sigma;
            obj.R = diag([0.01 0.01 0.01]);%zeros(3);
            obj.Q = diag([0.1 0.1 0.0001]);
        end
        
        function [obj,mu,Sigma] = step(obj,ut,landmark)
            %landmark(:,1:2) = landmark(:,1:2) + randn(size(landmark(:,1:2)))*obj.Q(1:2,1:2);
            [mu_bar,Sigma_bar] = obj.predict(ut);
            mu = mu_bar;
            Sigma = Sigma_bar;
            numLandmarks = size(landmark,1);
            ct = landmark(:,3);
            for i = 1:numLandmarks
                rt_i = landmark(i,1);
                phit_i = landmark(i,2);
                zt_i = [rt_i;phit_i;landmark(i,3)];
                j = ct(i);
                %disp(j);
                delta_x = obj.map(j,1)-mu_bar(1);
                delta_y = obj.map(j,2)-mu_bar(2);
                delta = [delta_x;delta_y];
                q = delta'*delta;
                zt_hat_i = [
                    sqrt(q);
                    atan2(delta_y,delta_x) - mu_bar(3);
                    obj.map(j,3)
                ];
                Ht_i = [
                    sqrt(q)*delta_x -sqrt(q)*delta_y 0;
                    delta_y         delta_x          -1;
                    0               0                0 
                ]/q;
                Kt_i = Sigma_bar*Ht_i'/(Ht_i*Sigma_bar*Ht_i' + obj.Q);
                disp("*")
                disp(zt_i-zt_hat_i);
                %if(~sum(isnan(Kt_i)))
                %mu = mu + Kt_i*(zt_i-zt_hat_i);
                Sigma = Sigma - Kt_i*Ht_i*Sigma_bar;
                %end
            end
            obj.mu = mu;
            obj.Sigma = Sigma;
        end
        
        function [mu_bar,Sigma_bar] = predict(obj,ut)
            v = ut(1); w = ut(2);
            dt = 0.05;
            theta = obj.mu(3);
            mu_bar = obj.mu + ...
                [
                  cos(theta)*v*dt;%-(v/w)*sin(theta) + (v/w)*sin(theta+w*dt);
                  sin(theta)*v*dt;%(v/w)*cos(theta) - (v/w)*cos(theta+w*dt);
                  w*dt
                ];
            Gt = [
              1 0 -v*sin(theta)*dt;%(v/w)*cos(theta) - (v/w)*cos(theta+w*dt);
              0 1 v*cos(theta)*dt;%(v/w)*sin(theta) - (v/w)*sin(theta+w*dt);
              0 0 1
            ];
            %disp(v/w)
            %disp(Gt);
            Sigma_bar = Gt*obj.Sigma*Gt' + obj.R;
        end
        
    end
end

