classdef fsm
    %FSM Abstract class for finite state machine
    
    properties
        numStates    % total number of states in the state machine
        states       % all the states
        transitions  % all the transitions between states
        currentState    % current state
    end
    
    methods (Abstract)
        compute_next_state(obj,input); % given input, return the next state
    end
end

