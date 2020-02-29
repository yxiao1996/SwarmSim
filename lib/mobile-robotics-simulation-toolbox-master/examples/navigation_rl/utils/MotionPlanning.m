classdef MotionPlanning < Simulink.IntEnumType
    % Enumeration datatype for motion planning status
    % Copyright 2019 The MathWorks, Inc.
    
    enumeration
        UNASSIGNED(0)
        PLANNING(1)
        FOLLOWING(2)
        SUCCESS(3)
        CANCELLED(4)
        FAILED(5)
    end
end