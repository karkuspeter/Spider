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
        actions  % each row is an action
        %skills   % numbers representing skills, i.e. action types
        thetas    % each column is control parameters for an action type
        
        mdp_T    % transfer function for mdp
        mdp_R
        
        num_states
        num_actions
    end
    
    methods
        function r = Raction(obj, state, next_state, action_idx)
            action = obj.actions(action_idx, :);
            
            centers = [0.3 0.6];
            vars = [0.05 0.03];
            f1 = @(x)(1*(1 - 0.9*gaussmf(x, [vars(1), centers(1)]) - 0.7*gaussmf(x, [vars(2), centers(2)])));
            f2 = @(x,slip)(1-1./(1+exp(-(x-0.4)*15*slip)));
            f =  @(x,slip)(f1(x) + f2(x, slip));
            slips = [0.1 0.9];  % values between 0 and 1
            
            slip = slips(obj.Tstate(state(1), state(2)));
            theta = obj.thetas{action(3)};
            r = -5*f(theta, slip);
%             if(obj.Tstate(state(1), state(2)) == action(3))
%                 r = 0;
%             else
%                 r = -1;
%             end
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
            trans = obj.actions(action,1:end-1);
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
        
        %% transpform to MDP formulation. T:[SxSxA] transition matrix, R:[SxSxA] reward matrix
        % only works for deterministic transition so far
        function [T, R] = mdp(obj)
            T = zeros(obj.num_states, obj.num_states, obj.num_actions);
            R = T;
            for i=1:obj.num_states
                sub = obj.stateval(i);
                if(obj.terminal(sub(1), sub(2)))
                    T(i, i, :) = ones(1,1,obj.num_actions);
                    R(i, i, :) = zeros(1,1,obj.num_actions);
                else
                    for a=1:obj.num_actions
                        [next_sub, r, ~] = obj.mdp_transit(sub, a);
                        next_i = obj.stateind(next_sub);
                        T(i, next_i, a) = 1;
                        R(i, next_i, a) = r;
                    end
                end
            end
        end
        
        %% value iteration
        function [Q, policy] = policy_iteration(obj, discount, policy0)
            if ~policy0
                policy0 = ones(obj.num_states,1);
            else
                policy0 = reshape(policy0, [numel(policy0), 1]);
            end
            [V, policy, iter, cpu] = mdp_policy_iteration(obj.mdp_T, obj.mdp_R, discount, policy0, 100,1);
            Q = squeeze(sum(obj.mdp_T.*(obj.mdp_R + discount.*repmat(V',obj.num_states, 1, obj.num_actions)), 2));

            [C,~]=max(Q,[],2);                              % finding the max values
            if(max(max(abs(C-V))) > 0.0001)
                error('Q and V mismatch')
            end
        end
        
        %% policy from Q function
        function [V, policy] = get_policy(obj, Q)
            [V,policy]=max(Q,[],2);
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
        
        %% display Q matrix
        function dispQ(obj, Q)
           % display the final Q matrix
            disp('Final Q matrix : ');
            disp(Q)
            [C,I]=max(Q,[],2);                              % finding the max values
            disp('Q(optimal):');
            disp(reshape(C, size(obj.Rstate)));
            disp('Optimal Policy');
            disp('*');
            disp(cellfun(@obj.action2str, num2cell(reshape(I, size(obj.Rstate))),'UniformOutput',false))
            %disp([action(I(2,1));action(I(3,1));action(I(4,1));action(I(5,1))]);
        end
    
    end
    
end


