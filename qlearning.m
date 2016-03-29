%% Q-learning with epsilon-greedy exploration Algorithm for Deterministic Cleaning Robot V1
%  Matlab code : Reza Ahmadzadeh
%  email: reza.ahmadzadeh@iit.it
%  March-2014
%% The deterministic cleaning-robot MDP
% a cleaning robot has to collect a used can also has to recharge its
% batteries. the state describes the position of the robot and the action
% describes the direction of motion. The robot can move to the left or to
% the right. The first (1) and the final (6) states are the terminal
% states. The goal is to find an optimal policy that maximizes the return
% from any initial state. Here the Q-learning epsilon-greedy exploration
% algorithm (in Reinforcement learning) is used.
% Algorithm 2-3, from:
% @book{busoniu2010reinforcement,
%   title={Reinforcement learning and dynamic programming using function approximators},
%   author={Busoniu, Lucian and Babuska, Robert and De Schutter, Bart and Ernst, Damien},
%   year={2010},
%   publisher={CRC Press}
% }
% notice: the code is written in 1-indexed instead of 0-indexed
%
% V1 the initial evaluation of the algorithm 
%

%% main function
% this will test the framework
function qlearning()
    params = struct('gamma', 0.5, ...       % discount factor  % TODO : we need learning rate schedule
                    'alpha', 0.5, ...       % learning rate    % TODO : we need exploration rate schedule
                    'epsilon', 0.9, ...     % exploration probability (1-epsilon = exploit / epsilon = explore)
                    'qlearn_iterations', 1000 ...
                    );

    world = WorldClass;

    world.Rstate = zeros(5,5);
    world.Rstate(2:4, 2:4) = -10;
    world.Rstate(5,3) = 10;

    world.Rinvalid = -10;
    
    world.Tstate = ones(size(world.Rstate));
    world.Tstate(:,1:2) = 2;

    %world.terminal = zeros(size(world.Rstate));
    world.terminal = world.Rstate~=0;

    %world.initial = zeros(size(world.Rstate));
    world.initial = world.Rstate==0;
    
    actions = [[0 1]; [0 -1]; [1 0]; [-1 0]];
    types = 3;
    for i=1:types
        world.actions = [world.actions; [actions repmat(i, size(actions,1), 1)]]
    end
    
    world.num_states = numel(world.Rstate);
    world.num_actions = size(world.actions,1);

    learn(world, params);
end

%% this is the main function including the initialization and the algorithm
% the inputs are: initial Q matrix, set of actions, set of states,
% discounting factor, learning rate, exploration probability,
% number of iterations, and the initial state.
function learn(world, params)
    % learning parameters
    gamma = params.gamma;    % discount factor  % TODO : we need learning rate schedule
    alpha = params.alpha;    % learning rate    % TODO : we need exploration rate schedule
    epsilon = params.epsilon;  % exploration probability (1-epsilon = exploit / epsilon = explore)

    % initial Q matrix
    Q = zeros(world.num_states, world.num_actions);
    % initialize with partial knowledge, i.e. Rstate values
    for x = 1:world.num_states
        for a = 1:world.num_actions
            [~, r, ~] = world.mdp_transit(world.stateval(x), world.actval(a));
            Q(x, a) = r;
        end
    end

    state_idx = world.samplestateind();
    %% the main loop of the algorithm
    for k = 1:params.qlearn_iterations
        disp(['iteration: ' num2str(k)]);

        % choose either explore or exploit
        r=rand; % get 1 uniform random number
        x=sum(r>=cumsum([0, 1-epsilon, epsilon])); % check it to be in which probability area
        if x == 1   % exploit
            [~,umax]=max(Q(state_idx,:));
            action_idx = umax;
        else        % explore
            action_idx = randi(world.num_actions); % choose 1 action randomly (uniform random distribution)
        end

        % observe the next state and next reward ** there is no reward matrix
        [next_state, r, terminal] = world.real_transit(world.stateval(state_idx), world.actval(action_idx));
        next_state_idx = world.stateind(next_state);

        % update the Q matrix using the Q-learning rule
        Q(state_idx,action_idx) = Q(state_idx,action_idx) + alpha * (r + gamma* max(Q(next_state_idx,:)) - Q(state_idx,action_idx));

        % print the results in each iteration
        %disp(['current state : ' num2str(state(state_idx)) ' next state : ' num2str(state(next_state_idx)) ' taken action : ' num2str(action(action_idx))]);
        disp([' next reward : ' num2str(r)]);
        %disp(Q);  % display Q in each level

        % if the robot is stuck in terminals
        if (terminal)
            state_idx = world.samplestateind();
        else
            state_idx = next_state_idx;
        end

    end
    
    % display the final Q matrix
    disp('Final Q matrix : ');
    disp(Q)
    [C,I]=max(Q,[],2);                              % finding the max values
    disp('Q(optimal):');
    disp(reshape(C, size(world.Rstate)));
    disp('Optimal Policy');
    disp('*');
    disp(cellfun(@world.action2str, num2cell(reshape(I, size(world.Rstate))),'UniformOutput',false))
    %disp([action(I(2,1));action(I(3,1));action(I(4,1));action(I(5,1))]);
    disp('*');
end
