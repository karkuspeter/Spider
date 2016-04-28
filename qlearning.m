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

%what should really happen:
%Q is initialized by V (policy iteration)
%every iteration: Q learning for a while (for how much?)
%policy search: take reward as difference from expected. collect samples,
%if we have enough do the update
% How to balance how
%much one skill is used vs the other? enforce exploration, initialize
%state from experience weighted sampling?

%todo: randomize goal state as well

% q learning should learn R and T to be general, now its only reaching a
% single goal

%% main function
% this will test the framework
function qlearning()
    params = struct('gamma', 0.9, ...       % discount factor  % TODO : we need learning rate schedule
                    'alpha', 0.9, ...       % learning rate    % TODO : we need exploration rate schedule
                    'epsilon', 0.4, ...     % exploration probability (1-epsilon = exploit / epsilon = explore)
                    'qlearn_iterations', 1000, ...
                    'r_samples', 1, ...
                    'theta_samples', 10, ...
                    'reweight_samples', 0, ...
                    'reps_epsilon', 0.6 ...
                    );

    world = WorldClass;

    world.Rstate = zeros(5,5);
    world.Rstate(2:4, 2:4) = -100;
    world.Rstate(5,3) = 10;

    world.Rinvalid = -100;
    
    world.Tstate = ones(size(world.Rstate));
    world.Tstate(:,1:2) = 2;

    %world.terminal = zeros(size(world.Rstate));
    world.terminal = world.Rstate~=0;

    %world.initial = zeros(size(world.Rstate));
    world.initial = world.Rstate==0;
    
    actions = [[0 1]; [0 -1]; [1 0]; [-1 0]];
    world.actions = [actions, 1*ones(size(actions, 1),1); 
                     actions, 2*ones(size(actions, 1),1); ];
    
    world.num_states = numel(world.Rstate);
    world.num_actions = size(world.actions,1);
    
    world.thetas = {0.32 0.65};
    
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
    % shold be state x action type
    [world.mdp_T, world.mdp_R] = world.mdp();
    [Q, policy] = world.policy_iteration(params.gamma, 0);
    world.dispQ(Q);
    Q_init = Q;
    %Q = zeros(world.num_states, world.num_actions);
    % initialize with partial knowledge, i.e. Rstate values
%     for x = 1:world.num_states
%         for a = 1:world.num_actions
%             [~, r, ~] = world.mdp_transit(world.stateval(x), world.actval(a));
%             Q(x, a) = r;
%         end
%     end

    %% the main Q learning loop
    state_idx = world.samplestateind();
    for k = 1:params.qlearn_iterations
        %disp(['iteration: ' num2str(k)]);

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
        [next_state, r, terminal] = world.real_transit(world.stateval(state_idx), action_idx);
        next_state_idx = world.stateind(next_state);

        % update the Q matrix using the Q-learning rule
        Q(state_idx,action_idx) = Q(state_idx,action_idx) + alpha * (r + gamma* max(Q(next_state_idx,:)) - Q(state_idx,action_idx));

        % print the results in each iteration
        %disp(['current state : ' num2str(state(state_idx)) ' next state : ' num2str(state(next_state_idx)) ' taken action : ' num2str(action(action_idx))]);
        %disp([' next reward : ' num2str(r)]);
        %disp(Q);  % display Q in each level

        % if the robot is stuck in terminals
        if (terminal)
            state_idx = world.samplestateind();
        else
            state_idx = next_state_idx;
        end

    end
    [V, policy] = world.get_policy(Q);
    world.dispQ(Q);
    
    %% main policy iteration loop
    mu = world.thetas;
    sigma = {0.05, 0.05};
    %world.thetas = normrnd(mu, sigma);  % sample all theta, even if some wont be used

    Dr = cell(2,1);
    Dtheta = cell(2,1);
    hist = cell(2,1);
    hist{1} = [hist{1}; mu{1}, sigma{1}];
    hist{2} = [hist{2}; mu{2}, sigma{2}];
    
    state_idx = world.samplestateind();
    for i=1:500
        % collect data for all controllers, update when enough data
        % available
        
        action_idx = policy(state_idx);
        action = world.actval(action_idx);
        type = action(3);
        % observe the next state and next reward ** there is no reward matrix
        [next_state, r, terminal] = world.real_transit(world.stateval(state_idx), action_idx);
        next_state_idx = world.stateind(next_state);
        
        expected_r = Q(state_idx, action_idx) - V(next_state_idx);
        r_gain = r - expected_r;
        
        Dr{type} = [Dr{type}; r_gain];
        Dtheta{type} = [Dtheta{type}; world.thetas{type}];
        
        % update theta distribution if needed (policy update step)
        if(length(Dr{type}) >= params.theta_samples)
            [mu{type}, sigma{type}] = reps_update(mu{type}, sigma{type}, Dtheta{type}, Dr{type}, params);
            %TODO reuse samples
            Dtheta{type} = [];
            Dr{type} = [];
            hist{type} = [hist{type}; mu{type}, sigma{type}];
        end
     
        % sample new theta if needed
        if(length(Dr{type}) >= params.r_samples)
            world.thetas{type} = normrnd(mu{type}, sigma{type});
        end
        
        if (terminal)
            state_idx = world.samplestateind();
        else
            state_idx = next_state_idx;
        end
        
        %w_hist = [w_hist; mu sigma];
        %Rmean_hist = [Rmean_hist; mean(R_hist(end-theta_samples+1:end))];

    end
    hist{1}
    hist{2}
end


function [mu, sigma] = reps_update(mu, sigma, Dtheta, Dr, params)
        epsilon = params.reps_epsilon;
        
        %compute weights
        Dw = zeros(size(Dtheta,1),1);
        if (params.reweight_samples)
            for i=1:min(policy_samples, size(Dw,1)/theta_samples)
                dex = (i-1)*theta_samples+1:(i)*theta_samples;
                prob_sample = prod(normpdf(Dtheta(dex,:), repmat(w_hist(end-i+1, 1:thetadim), theta_samples, 1), repmat(w_hist(end-i+1, thetadim+1:end), theta_samples, 1)),2);
                prob_current = prod(normpdf(Dtheta(dex,:), repmat(mu, theta_samples, 1), repmat(sigma, theta_samples, 1)),2);
                Dw(dex, 1) = prob_current./prob_sample;
                Dw(dex, 1) = Dw(dex, 1)/sum(Dw(dex, 1));
            end
            Dw = Dw / min(policy_samples, size(Dw,1)/theta_samples);
        else
            % no weights
            Dw = 1/size(Dr,1)*ones(size(Dr));
            % fixed weight
            %Dw = 1/10*ones(size(Dr));
        end
        
        % dual function
        Z = @(eta)exp((Dr-max(Dr))/eta);
        g_fun = @(eta) eta*epsilon + max(Dr) + eta .* log(sum(Z(eta).*Dw));
        deta_fun = @(eta) epsilon + log(sum(Z(eta).*Dw)) - sum(Z(eta).*(Dr-max(Dr)))./(eta*sum(Z(eta)));
        deal2 = @(varargin) deal(varargin{1:nargout});
        opt_fun = @(eta) deal2(g_fun(eta), deta_fun(eta));
        
        eta_star = fmincon(opt_fun, [0.001], [-1], [0], [], [], [], [], [], optimset('Algorithm','interior-point', 'GradObj','on', 'MaxFunEvals', 500, 'Display', 'off'));
        
        Z = Z(eta_star);
        Z_ext = repmat(Z,[1, size(Dtheta,2)]);
        
        mu = sum(Z_ext.*Dtheta)/sum(Z);
        denom = ((sum(Z).^2 - sum(Z.^2))/sum(Z));
        sigma = sqrt(sum(Z_ext.*((Dtheta-repmat(mu,[size(Dtheta,1),1])).^2))/denom);
        
end
