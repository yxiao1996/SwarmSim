classdef KalmanFilterDiffDrive
    %KALMANFILTER 
    % implementation of classical kalman filter for linear mode and
    % Gaussain noise for differential drive dynamics
    
    properties
        mu
        Sigma
        R
        Q
    end
    
    methods
        function obj = KalmanFilterDiffDrive(x)
            %KALMANFILTER 
            % initialize the kalman filter at x with high confidence
            obj.mu = x;
            obj.Sigma = diag([0.1 0.1 0.1]);
            obj.R = zeros(3);
            obj.Q = diag([0.1 0.1 0.1]);
        end
        
        function [obj,mu_pred,Sigma_pred] = step(obj,ut,zt)
            % step the kalman filter with control and measurement
            [mu_bar,Sigma_bar] = obj.predict(ut);
            obj = obj.update(mu_bar,Sigma_bar,zt);
            mu_pred = obj.mu;
            Sigma_pred = obj.Sigma;
        end
        
        function [mu_bar,Sigma_bar] = predict(obj,ut)
            A = eye(3);
            phi = obj.mu(3);
            B = [
              cos(phi) 0;
              sin(phi) 0;
              0        1
            ]*0.05;
            mu_t_1 = obj.mu;
            Sigma_t_1 = obj.Sigma;
            mu_bar = A*mu_t_1 + B*ut;
            Sigma_bar = A*Sigma_t_1*A + obj.R;
        end
        
        function obj = update(obj,mu_bar,Sigma_bar,zt)
            C = eye(3);
            Kt = Sigma_bar*C'/(C*Sigma_bar*C'+obj.Q); % kalman gain
            obj.mu = mu_bar+Kt*(zt-C*mu_bar);
            obj.Sigma = (eye(3)-Kt*C)*Sigma_bar;
        end
    end
end

