classdef DifferentialDrive < handle
    % DIFFERENTIALDRIVE Differential Drive robot utilities
    %
    % For more information, see <a href="matlab:edit mrsDocDifferentialDrive">the documentation page</a>
    %
    % Copyright 2018 The MathWorks, Inc.
    
    properties
        wheelRadius = 0.1;  % Wheel radius [m]
        wheelBase = 0.5;    % Wheelbase [m]
    end
    
    methods
        function obj = DifferentialDrive(wheelRadius,wheelBase)
            % DIFFERENTIALDRIVE Construct an instance of this class
            obj.wheelRadius = wheelRadius;
            obj.wheelBase = wheelBase;
        end
        
        function [v,w] = forwardKinematics(obj,wL,wR)
            % Calculates linear and angular velocity from wheel speeds
            v = 0.5*obj.wheelRadius*(wL+wR);
            w = (wR-wL)*obj.wheelRadius/obj.wheelBase;
        end
              
        function [wL,wR] = inverseKinematics(obj,v,w)
           % Calculates wheel speeds from linear and angular velocity
           wL = (v - w*obj.wheelBase/2)/obj.wheelRadius;
           wR = (v + w*obj.wheelBase/2)/obj.wheelRadius;
        end
        
    end
end

