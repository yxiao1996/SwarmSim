classdef FourWheelSteering < handle
    % FOURWHEELSTEERING Utilities for four-wheeled vehicle with front and
    % rear steering.
    % Ackermann steering is assumed, so the kinematic model is approximated
    % with a two-wheeled bicycle model with front and rear steering.
    % Source: https://www.ntu.edu.sg/home/edwwang/confpapers/wdwicar01.pdf
    %
    % For more information, see <a href="matlab:edit mrsDocFourWheelSteering">the documentation page</a>
    %
    % Copyright 2018 The MathWorks, Inc.
    
    properties
        wheelRadius = 0.1;  % Wheel radius [m]
        frontWheelDist = 0.25; % CG to Front Wheel Distance [m]
        rearWheelDist = 0.25; % CG to Rear Wheel Distance [m]
    end
    properties (Access='private')
        tolerance = 1e-12; % Tolerance used to avoid divisions by zero
    end
    
    methods
        function obj = FourWheelSteering(wheelRadius,wheelDistances)
            % FOURWHEELSTEERING Construct an instance of this class
            
            % Unpack parameters
            obj.wheelRadius = wheelRadius;
            % Accepts either a 1 or 2-element array of wheel distances
            % In the 1-element case, front and rear wheel distances are
            % set to the value provided.
            if numel(wheelDistances) == 1
                obj.frontWheelDist = wheelDistances;
                obj.rearWheelDist = wheelDistances;
            else
                obj.frontWheelDist = wheelDistances(1);
                obj.rearWheelDist = wheelDistances(2);
            end
        end
        
        function bodySpeeds = forwardKinematics(obj,wheelSpeeds,steerAngles)
            % Calculates linear and angular velocities [vx;vy;w],
            % in the *body* frame, from wheel speeds [wFront;wRear] and
            % steering angles [phiFront;phiRear]
            wFront = wheelSpeeds(1);
            wRear = wheelSpeeds(2);
            phiFront = steerAngles(1);
            phiRear = steerAngles(2);
            
            vx = 0.5*obj.wheelRadius*(wFront*cos(phiFront) + wRear*cos(phiRear));
            vy = 0.5*obj.wheelRadius*(wFront*sin(phiFront) + wRear*sin(phiRear));
            w = (wFront*sin(phiFront)-wRear*sin(phiRear))* ...
                obj.wheelRadius/(obj.frontWheelDist+obj.rearWheelDist);
            bodySpeeds = [vx;vy;w];
        end
              
        function [wheelSpeeds,steerAngles] = inverseKinematicsFrontSteer(obj,vx,w)
            % Calculates wheel speeds [wFront;wRear] and steer angles 
            % [phiFront;phiRear] from x velocity vx and angular velocity w
            % in the *body* frame. Assumes no rear steering and equal front
            % and rear wheel speeds.
           
            % Calculate front steer angle
            if abs(vx) > obj.tolerance
                % NOTE: Using atan instead of atan2 because we want steer
                % angle to be two-quadrant, in the range [-pi/2,pi/2]
                phiFront = atan(w*(obj.frontWheelDist+obj.rearWheelDist)/vx);
            else
                phiFront = 0; % Handle zero-speed case, which causes division by zero
            end
                        
            % Calculate wheel speeds
            vRear = vx; % Rear wheel matches forward speed
            vFront = vx/cos(phiFront); % Front wheel matches forward speed given steer angles
            
            % Pack the outputs
            wheelSpeeds = [vFront;vRear]/obj.wheelRadius; 
            steerAngles = [phiFront;0]; % Zero rear steering
        end

        function [wheelSpeeds,steerAngles] = inverseKinematicsZeroSideslip(obj,vx,w)
            % Calculates wheel speeds [wFront;wRear] and steer angles 
            % [phiFront;phiRear] from x velocity vx and angular velocity w
            % in the *body* frame. Assumes equal front and rear wheel
            % speeds, as well as the front and rear steering angles being
            % opposite such that these is zero sideslip.
           
            % Calculate steer angle
            if abs(vx) > obj.tolerance
                % NOTE: Using atan instead of atan2 because we want steer
                % angle to be two-quadrant, in the range [-pi/2,pi/2]
                phi = atan(w*(obj.frontWheelDist+obj.rearWheelDist)/vx);
            else
                phi = 0; % Handle zero-speed case, which causes division by zero
            end
                        
            % Calculate wheel speeds
            vWheel = vx/cos(phi);
            
            % Pack the outputs
            wheelSpeeds = [vWheel;vWheel]/obj.wheelRadius; % Equal wheel speeds
            steerAngles = [phi;-phi]; % Equal and opposite steer angles
        end

        function [wheelSpeeds,steerAngles] = inverseKinematicsParallelSteer(obj,vx,vy)
            % Calculates wheel speeds [wFront,wRear] and steer angles 
            % [phiFront;phiRear] from linear velocities [vx;vy]
            % in the *body* frame. Assumes equal front and rear steering 
            % angles so the vehicle moves linearly without rotating.
           
            % Calculate steer angle
            if abs(vx) > obj.tolerance
                % NOTE: Using atan instead of atan2 because we want steer
                % angle to be two-quadrant, in the range [-pi/2,pi/2]
                phi = atan(vy/vx);
            else
                % Set steer angle to +pi/2, -pi/2, or zero based on desired y speed
                phi = (pi/2)*sign(vy); 
            end
                        
            % Calculate wheel speeds
            % Turn wheels forward or backward based on X direction of speed
            vWheel = sqrt(vx^2 + vy^2)*sign(vx);
            
            % Pack the outputs
            wheelSpeeds = [vWheel;vWheel]/obj.wheelRadius; % Equal wheel speeds
            steerAngles = [phi;phi]; % Equal steer angles
        end        
        
    end
end

