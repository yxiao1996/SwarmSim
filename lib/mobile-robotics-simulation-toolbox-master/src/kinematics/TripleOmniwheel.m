classdef TripleOmniwheel < handle
    % TRIPLEOMNIWHEEL Triple Omniwheel Drive robot utilities
    %
    % For more information, see <a href="matlab:edit mrsDocTripleOmniwheel">the documentation page</a>
    %
    % Copyright 2018 The MathWorks, Inc.
    
    properties
        wheelRadius = 0.1;  % Wheel radius [m]
        robotRadius = 0.25; % Robot radius [m]
        wheelAngles = [0, 2*pi/3, -2*pi/3]; % Wheel angles relative to x-axis [rad]
    end
    properties(Access='private')
        fwdMatrix = eye(3); % Transformation matrix from wheel speeds to body speeds
        invMatrix = eye(3); % Transformation matrix from body speeds to wheel speeds 
    end
    
    methods
        function obj = TripleOmniwheel(wheelRadius,robotRadius,wheelAngles)
            % TRIPLEOMNIWHEEL Construct an instance of this class
            
            % Unpack parameters
            obj.wheelRadius = wheelRadius;
            obj.robotRadius = robotRadius;
            obj.wheelAngles = wheelAngles;
            
            % Create transformation matrices
            computeMatrices(obj);           
        end
        
        function bodySpeeds = forwardKinematics(obj,wheelSpeeds)
            % Calculates linear and angular velocities [vx;vy;w],
            % in the *body* frame, from wheel speeds [w1;w2;w3]
            bodySpeeds = obj.fwdMatrix*wheelSpeeds;
        end
                     
        function wheelSpeeds = inverseKinematics(obj,bodySpeeds)
           % Calculates wheel speeds [w1;w2;w3] from linear and 
           % angular velocities [vx;vy;w] in the *body* frame
           wheelSpeeds = obj.invMatrix*bodySpeeds;
        end
               
        function M = getForwardMatrix(obj)
            % Returns forward kinematics matrix (wheel speeds to body speeds)
            M = obj.fwdMatrix;
        end
        
        function M = getInverseMatrix(obj)
            % Returns forward kinematics matrix (wheel speeds to body speeds)
            M = obj.invMatrix;
        end
        
        function computeMatrices(obj)
            % Creates the forward and inverse matrices given the object's
            % kinematic properties
            
            obj.fwdMatrix = [ sin(obj.wheelAngles); ...
                             -cos(obj.wheelAngles); ...
                             -[1, 1, 1]/obj.robotRadius] * obj.wheelRadius/3;
            obj.invMatrix = inv(obj.fwdMatrix);
            
        end
        
    end
end

