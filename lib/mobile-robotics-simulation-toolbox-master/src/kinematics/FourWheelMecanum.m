classdef FourWheelMecanum < handle
    % FOURWHEELMECANUM Utilities for four-wheeled vehicle with individually
    % driven Mecanum wheels (also known as Swedish or Ilon wheels).
    % The ordering of the wheels is Front Left, Front Right, Rear Left,
    % Rear Right.
    % Source: http://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
    %
    % For more information, see <a href="matlab:edit mrsDocFourWheelMecanum">the documentation page</a>
    %
    % Copyright 2018 The MathWorks, Inc.
    
    properties
        wheelRadius = 0.1;  % Wheel radius [m]
        wheelBase = 0.4;    % Wheelbase [m]
        wheelTrack = 0.5;   % Wheel track [m]
    end
    properties(Access=private)
        fwdMatrix = zeros(3,4); % Transformation matrix from wheel speeds to body speeds
        invMatrix = zeros(4,3); % Transformation matrix from body speeds to wheel speeds
    end
    
    methods
        
        function obj = FourWheelMecanum(wheelRadius,wheelBase,wheelTrack)
            % FOURWHEELMECANUM Construct an instance of this class
            
            % Unpack parameters
            obj.wheelRadius = wheelRadius;
            obj.wheelBase = wheelBase;
            obj.wheelTrack = wheelTrack;
            
            % Create forward and inverse matrices
            computeMatrices(obj);
            
        end
        
        function bodySpeeds = forwardKinematics(obj,wheelSpeeds)
            % Calculates linear and angular velocities [vx;vy;w],
            % in the *body* frame, from wheel speeds [w1;w2;w3;w4]
            bodySpeeds = obj.fwdMatrix * wheelSpeeds;
        end
              
        function wheelSpeeds = inverseKinematics(obj,bodySpeeds)
            % Calculates wheel speeds [w1;w2;w3;w4] from linear and 
            % angular velocities [vx;vy;w] in the *body* frame
            wheelSpeeds = obj.invMatrix * bodySpeeds;
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
            
            R = (obj.wheelBase + obj.wheelTrack)/2;
            % Create forward kinematics matrix (wheel speeds to body speeds)
            obj.fwdMatrix = [ 1, 1, 1, 1; ... 
                             -1, 1, 1,-1; ... 
                             -1/R, 1/R, -1/R, 1/R ] * (obj.wheelRadius/4);

            % Create inverse kinematics matrix (body speeds to wheel speeds)
            obj.invMatrix = [ 1, -1, -R; ... 
                              1,  1,  R; ...
                              1,  1, -R; ...
                              1, -1,  R ] / obj.wheelRadius; 
        end
        
    end
end

