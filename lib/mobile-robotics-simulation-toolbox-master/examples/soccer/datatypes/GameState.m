classdef GameState < Simulink.IntEnumType
    % Enumeration datatype for Robot Soccer game state
    % Copyright 2019 The MathWorks, Inc.
    
    enumeration
        InPlay(1)
        OutOfPlay(2)
        GoalScored(3)
    end
    
end