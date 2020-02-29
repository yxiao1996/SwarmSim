classdef GenericOmniwheel < handle
    % GENERICOMNIWHEEL Utilities for omniwheel vehicles with generic
    % number, position, and orientation of wheels.
    %
    % For more information, see <a href="matlab:edit mrsDocGenericOmniwheel">the documentation page</a>
    %
    % Copyright 2018 The MathWorks, Inc.
    
    properties
        wheelRadius = 0.1;           % Wheel radius [m]
        wheelPositions = [1 1;1 -1]; % Wheel (X,Y) positions relative to CG [m]
        wheelAngles = [0;0];         % Wheel angles relative to CG [rad]
    end
    properties(Access=private)
        fwdMatrix = zeros(3,4); % Transformation matrix from wheel speeds to body speeds
        invMatrix = zeros(4,3); % Transformation matrix from body speeds to wheel speeds
    end
    
    methods
        
        function obj = GenericOmniwheel(wheelRadius,wheelPositions,wheelAngles)
            % GENERICOMNIWHEEL Construct an instance of this class
            
            % Unpack parameters
            obj.wheelRadius = wheelRadius;
            obj.wheelPositions = wheelPositions;
            obj.wheelAngles = wheelAngles;
            
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
            % NOTE: This uses the pseudoinverse so it may not be accurate
            wheelSpeeds = obj.invMatrix * bodySpeeds;
        end
              
        function M = getForwardMatrix(obj)
            % Returns forward kinematics matrix (wheel speeds to body speeds)
            M = obj.fwdMatrix;
        end
        
        function M = getInverseMatrix(obj)
            % Returns inverse kinematics matrix (wheel speeds to body speeds)
            M = obj.invMatrix;
        end
        
        function computeMatrices(obj)
            % Creates the forward and inverse matrices given the object's
            % kinematic properties
            
            % Initialize the forward kinematics matrix
            numWheels = size(obj.wheelPositions,1);
            M = zeros(3,numWheels);
            
            % If the wheel angles property is scalar, replicate it to all
            % the wheels on the vehicle.
            if isscalar(obj.wheelAngles) && numWheels > 1
                alpha = repmat(obj.wheelAngles,[numWheels 1]);
            else
                alpha = obj.wheelAngles; 
            end
            
            % Loop through each wheel and calculate the forward dynamics
            for idx = 1:numWheels
               
                % Linear components
                M(1,idx) = cos(alpha(idx)); % X direction
                M(2,idx) = sin(alpha(idx)); % Y direction
                
                % Angular component
                % If the line is close to vertical, slope is infinite so
                % this is a special case of the calculation that only
                % requires the X distance
                if abs( abs(alpha(idx)) - pi/2 ) < 1e-3
                    M(3,idx) = obj.wheelPositions(idx,1);
                % Else, do the full calculation with line equation
                % (slope + intercept)
                else
                    slope = tan(alpha(idx));
                    intercept = obj.wheelPositions(idx,2) - slope*obj.wheelPositions(idx,1);
                    M(3,idx) = abs(intercept)/sqrt(1+slope^2);
                end
                
            end
            
            % Multiply by wheel radius to convert from angular to linear 
            % speed, and average over the number of wheels
            obj.fwdMatrix = M * obj.wheelRadius / numWheels;
            
            % Use pseudoinverse to *approximate* the inverse kinematics matrix
            obj.invMatrix = pinv(obj.fwdMatrix);
            
        end
        
    end
end

