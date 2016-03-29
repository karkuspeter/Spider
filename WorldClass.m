%% class for world
classdef WorldClass
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Rstate   % reward arriving at state
        Rinvalid % reward for invalid actions
        Tstate   % type of state
        terminal % 1 for terminal states
        initial  % 1 for valid initial states
        actions  % each row is an action, last column represents action type
        theta    % control parameter
        
        num_states
        num_actions
    end
    
    methods
        function vect = effect(obj, action)
            vect = action(1, 1:2);
        end
        
        function r = Raction(obj, state, next_state, action)
            %r = -1; % fix for now
            if(obj.Tstate(state(1), state(2)) == action(3))
                r = -1;
            else
                r = -2;
            end
            %r = func(theta, Tstate(state), action(3))
        end
        
        function state = stateval(obj, ind)
            [x1, x2] = ind2sub(size(obj.Rstate), ind);
            state = [x1 x2];
        end
        function act = actval(obj, ind)
             act = obj.actions(ind, :);
        end    
        function ind = stateind(obj, state)
            ind = sub2ind(size(obj.Rstate), state(1), state(2));
        end
        %function ind = actind(obj, sub)
        %    ind = sub2ind(size(obj.actions), sub);
        %end 

        %% This function is used as an observer to give the next state and the next reward using the current state and action
        function [next_state, r, terminal] = mdp_transit(obj, state, action)
            trans = obj.effect(action);
            next_state = state + trans;
            next_idx = num2cell(next_state);

            for dim=1:2
                if( next_state(dim) <= 0 || next_state(dim) > size(obj.Rstate, dim) )
                    next_state = state;
                    next_idx = num2cell(next_state);
                    r = obj.Rinvalid;
                    terminal = obj.terminal(next_idx{:});
                    return
                end
            end

            r = obj.Rstate(next_idx{:});
            terminal = obj.terminal(next_idx{:});
        end

        %% This function is used as an observer to give the next state and the next reward using the current state and action
        function [next_state, r, terminal] = real_transit(obj, state, action)
            [next_state, r, terminal] = obj.mdp_transit(state, action);
            r = r + obj.Raction(state, next_state, action);
        end
    
        %% sample initial state, return index
        function stateind = samplestateind(obj)
            % rejection sampling
            while(true)
                row = randi(size(obj.Rstate,1));
                col = randi(size(obj.Rstate,2));
                if(obj.initial(row, col))
                    stateind = obj.stateind([row, col]);
                    return;
                end
            end
        end
        
        %% display action
        function str = action2str(obj, action)
            act = obj.actions(action,:);
            str = ' ';
            if (act(1) < 0)
                str = strcat(str,'U');
            elseif (act(1) > 0)
                str = strcat(str,'D');
            end
            if (act(2) < 0)
                str = strcat(str,'L');
            elseif (act(2) > 0)
                str = strcat(str,'R');
            end
            str = strcat(str, num2str(act(3)));
            str = strcat(str, ' ');
        end
    end
    
end

