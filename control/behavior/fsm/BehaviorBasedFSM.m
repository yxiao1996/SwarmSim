classdef BehaviorBasedFSM < fsm
    %BEHAVIORBASEDFSM 
    % finite state machine for behavior-based control
    
    properties
    end
    
    methods
        function obj = BehaviorBasedFSM(initialState)
            %BEHAVIORBASEDFSM constuctor
            obj.states = ["go-to-goal";"avoid-wall"];
            if(~ismember(initialState,obj.states))
                msg = "finite state machine: invalid initial state!";
                error(msg);
            end
            obj.currentState = initialState;
        end
        
        function [next_state,obj] = compute_next_state(obj,reads,direc_avoid,direc_goal)
            switch obj.currentState 
                case "go-to-goal"
                    % check for obstacles in the environment
                    dist_obst = sqrt(dot(direc_avoid,direc_avoid));
                    if (dist_obst > 0.1 && dot(direc_avoid,direc_goal)<=0)
                        obj.currentState = "avoid-wall";
                    end
                case "avoid-wall"
                    dist_obst = sqrt(dot(direc_avoid,direc_avoid));
                    if (dist_obst <= 0.1 || dot(direc_avoid,direc_goal)>0)
                        obj.currenState = "go-to-goal";
                    end
                otherwise
                    obj.currenState = obj.currentState;
            end
            next_state = obj.currentState;
        end
    end
end

