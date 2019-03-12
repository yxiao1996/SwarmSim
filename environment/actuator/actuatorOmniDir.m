classdef actuatorOmniDir < actuator
    %ACTUATOROMNIDIR 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        R
        L
    end
    
    methods
        function obj = actuatorOmniDir(R,L)
            %ACTUATOROMNIDIR omni-directional dynamics
            wheelRad = R;
            e = L*sqrt(3)/2;
            wheelPos = [e e;-e e;-e -e;e -e];
            wheelAng = [0, pi/4, -pi/4, 0];
            obj.dynamics = GenericOmniwheel(wheelRad,wheelPos,wheelAng);
        end
        
        function vel = actuate(obj,control,pose)
            % actuate the vehicle to move
            xRef = control.xRef;
            yRef = control.yRef;
            wRef = control.wRef;
            % inverse kinematics
            velRef = [xRef;yRef;wRef];
            w = inverseKinematics(obj.dynamics,velRef);
            % forward kinematics
            velB = forwardKinematics(obj.dynamics,w);
            vel = bodyToWorld(velB,pose);  % Convert from body to world
        end
    end
end

