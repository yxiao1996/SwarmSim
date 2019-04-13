classdef EKFLocalizerCorrespondences
    %EKFLOCALIZERCORRESPONDENCES 
    %
    
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
            obj.Q = diag([0.1 0.1 0.001]);
        end
        
        function [obj,mu,Sigma] = step(obj,ut,landmark,pose)
            %landmark(:,1:2) = landmark(:,1:2) + randn(size(landmark(:,1:2)))*obj.Q(1:2,1:2);
            [mu_bar,Sigma_bar] = obj.predict(ut);
            mu = mu_bar;
            Sigma = Sigma_bar;
            numLandmarks = size(landmark,1);
            disp(landmark);
            for i = 1:numLandmarks
                ct = landmark(:,3);
                rt_i = landmark(i,1);
                phit_i = landmark(i,2);
                zt_i = [rt_i;phit_i;landmark(i,3)];
                j = ct(i);
                delta_x = obj.map(j,1)-mu_bar(1);
                delta_y = obj.map(j,2)-mu_bar(2);
                delta = [delta_x;delta_y];
                delta_ = [obj.map(j,1)-pose(1);obj.map(j,2)-pose(2)];
                
                zt_real_i = [
                    sqrt(dot(delta_,delta_));
                    obj.angle_diff(atan2(delta_(2),delta_(1)), pose(3));
                ];
                assert(j==obj.map(j,3));
                disp("observed position")
                disp(zt_i);
                disp("position on map")
                disp([obj.map(j,1) obj.map(j,2)]);
                disp("real relative position")
                disp(zt_real_i);
                disp("difference between real and estimate")
                disp(zt_i(1:2)-zt_real_i);
                assert(zt_i(1)-zt_real_i(1)<0.0001);
                % hold on; plot(obj.map(j,1),obj.map(j,2)); hold off;
                q = delta'*delta;
                zt_hat_i = [
                    sqrt(q);
                    %atan2(delta_y,delta_x) - mu_bar(3);
                    obj.angle_diff(atan2(delta_y,delta_x),mu_bar(3));
                    obj.map(j,3)
                ];
                Ht_i = [
                    sqrt(q)*delta_x -sqrt(q)*delta_y 0;
                    delta_y         delta_x          -1
                    %0               0                0 
                ]/q;
                Kt_i = Sigma_bar*Ht_i'/(Ht_i*Sigma_bar*Ht_i' + obj.Q(1:2,1:2));
                
                %disp(delta-delta_);
                %if(~sum(isnan(Kt_i)))
                mu = mu + Kt_i*(zt_i(1:2)-zt_hat_i(1:2));
                %mu = mu + Kt_i*(zt_i(1:2)-zt_real_i(1:2));
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
                  cos(theta)*v*dt;%
                  %-(v/w)*sin(theta) + (v/w)*sin(theta+w*dt);
                  sin(theta)*v*dt;%
                  %(v/w)*cos(theta) - (v/w)*cos(theta+w*dt);
                  w*dt
                ];
            Gt = [
              1 0 -v*sin(theta)*dt;%
              %1 0 (v/w)*cos(theta) - (v/w)*cos(theta+w*dt);
              0 1 v*cos(theta)*dt;%
              %0 1 (v/w)*sin(theta) - (v/w)*sin(theta+w*dt);
              0 0 1
            ];
            %disp(v/w)
            %disp(Gt);
            Sigma_bar = Gt*obj.Sigma*Gt' + obj.R;
        end
        
        function a = angle_diff(obj,a1,a2)
            v = (cos(a1)+1j*sin(a1)) / (cos(a2)+1j*sin(a2));
            a = angle(v);
        end
        
    end
end

