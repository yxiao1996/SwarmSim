classdef RobotAction < Simulink.IntEnumType
    % Enumeration datatype for Robot Soccer robot actions
    % Copyright 2019 The MathWorks, Inc.
    
    enumeration
        HoldBall(1)
        KickBall(2)
        DoNothing(3)
    end
    
end