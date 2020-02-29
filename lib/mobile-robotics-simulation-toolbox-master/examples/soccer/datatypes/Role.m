classdef Role < Simulink.IntEnumType
    % Enumeration datatype for Robot Soccer robot roles
    % Copyright 2019 The MathWorks, Inc.
    
    enumeration
        Attacker(1)
        Defender(2)
        Goalkeeper(3)
        None(4)
    end
end