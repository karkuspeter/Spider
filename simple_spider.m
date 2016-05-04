% load environment
function simple_spider()

if ~exist('minimize')
    run ../gpml-matlab-v3.6-2015-07-07/startup.m;
    addpath ../SPGP_dist;
end






% initialize parameters
params = struct('thetadim', 0, ...
                'slip_samples', 0, 'theta_samples', 0, 'iterations', 0, ...
                'reps_epsilon', 0, 'reps_reweight', 0, ...
                'mu', 0, 'sigma', 0, ...
                'plan_off', 0 );

if ~exist('output_off')
    output_off = 0;
end

params.thetadim = 3;

params.slip_samples=8;
params.theta_samples=20;
params.iterations=10000;

params.reps_epsilon = 0.60;
params.reps_reweight = 0;

params.mu = -0.5 * ones(1, params.thetadim);
params.sigma = 1 * ones(1, params.thetadim);

params.plan_off = 0;


R_hist = [];
Rmean_hist = [];
theta_hist = [];
trans_hist = [];
w_hist = [];
guess_hist = [];
total_samples = 0;


%% start of new script

ctr_struct = struct('N_theta', 0, ... % number of theta samples collected before update
                    'N_slip', 0, ...  % number of slip samples collected for a single theta
                    'mu', 0, ...
                    'sigma', 0, ...
                    'theta', 0, ...
                    'D', zeros(params.slip_samples*params.theta_samples,params.thetadim + 2, 1) ); %slip_sample*theta_sample x data x policy
ctr_struct.mu = params.mu;
ctr_sturct.sigma = params.sigma;
                
ctrls = {ctr_struct, ctr_struct};
ctrls{1}.theta = normrnd(params.mu, params.sigma);
ctrls{2}.theta = normrnd(params.mu, params.sigma);

Ps_est = [0; 0];

for iter = 1:params.iterations
    
    state = randi(3); % sample init state
    % compute policy (enough for this state)
    Test = Transition_function(Ps_est);
    Rest = R_function(Ps_est);
    [vals, policy] = min(sum(Test.*Rest,2), [], 3);
    action = policy(state);
    % make a deep copy of controller struct and overwrite in the end
    % this would be shallow copy i believe if ctrl is a class 
    ctr = ctrls{action};
    % naive version, dont consider what is better to learn
    
    % execute with real MDP
    Ps_real = Slip_function([ctrls{1}.theta; ctrls{2}.theta]);
    Treal = Transition_function(Ps_real);
    Rreal = R_function(Ps_real);
    outcome = randsample(5, 1, true, Treal(state, :, action)');
    reward = Rreal(state, outcome, action);
    slipped = (outcome ~= action);
    
    % use this for learning the used controller
    ctr.D(ctr.N_theta*params.theta_samples + ctr.N_slip+1,:,end) = [ctr.theta, reward, slipped];
    ctr.N_slip = ctr.N_slip + 1;
    
    % sample new theta or update policy if enough samples collected
    if ctr.N_slip >= params.slip_samples
        ctr.N_slip = 0;
        ctr.N_theta = ctr.N_theta + 1;
        % update policy distr if enough theta samples collected
        if ctr.N_theta >= params.theta_samples
            ctr.N_theta = 0;
            [ctr.mu, ctr.sigma] = reps_update(ctr.mu, ctr.sigma, ctr.D(:,:, end), params);
            % expand D matrix
            ctr.D(:,:,end+1) = 0;
        end
        % sample theta from policy distribution
        ctr.theta = normrnd(ctr.mu, ctr.sigma);
    end
    
    % update slip estimation
    if size(ctr.D, 4) > 1
        slips_prev = reshape(ctr.D(:,3,:,end-1), [], 1);
        slips_this = reshape(ctr.D(1:ctr.N_slips, 3, 1:ctr.N_theta+1, end), [], 1);
        Ps_est(action) = mean([slips_prev; slips_this]);
    end
           
    ctrls{action} = ctr;
end

%compute stats
stats = [];
for i=1:2
    ctr = ctrls{i};
    stat = struct('thetas', [], ...
                  'rewards', [], ...%zeros(params.theta_samples,params.thetadim, size(ctr.D,3)), ...
                  'slips', []);
    stat.thetas = ctr.D(1:params.slip_samples:end, 1:params.thetadim, :);
    stat.rewards = permute(mean(ctr.D(:, params.thetadim+1, :),1), [3 2 1]);
    stat.slips = permute(mean(ctr.D(:, params.thetadim+2, :),1), [3 2 1]);
    
    stats = [stats; stat];
end

%% old code
% for i=1:1
%     for j=1:theta_samples
%         % sample theta
%         theta = normrnd(mu, sigma);
% 
%         R = [];
%         R_model = [];
%         transitions = [];
%         Pslip = init_p;
%         R_est = 0;
%         prev_plan = init_plan;
%         while size(R,1) < R_samples
%             % plan 
%             if(params.plan_off)
%                 u_plan = bridge_plan;
%                 plan_raw = init_plan;
%             else
%                 %Pslip = max(0,model.f(model, theta));
%                 [u_plan, plan_raw] = plan(world, Pslip, prev_plan, R_est);
%             end
% 
%             if ~isequal(plan_raw, prev_plan)
%                 wasted_plans = wasted_plans + i;
%                 i = 1;
%                 R = [];
%                 R_model = [];
%                 %transitions = [];
%             end
%             % execute plan, get real world experience
%             [Ri, ti, Rnomi] = execute(world, x0, u_plan, theta, params);
%             R = [R; Ri];
%             R_model = [R_model; Rnomi];
%             transitions = [transitions; ti];
%             
%             prev_plan = plan_raw;
%             Pslip = sum(transitions)/size(transitions,1)/2;
%             R_est = (Ri-Rnomi)*params.trans_cheat/length(ti);
%             % now estimate is perfect, but afterwards should use history
% %             if abs(R_est - params.R_func(0, theta)) > 0.0001
% %                 R_est
% %             end
%             %should be weighted mean with length of transitions
% 
%         end
%         if length(R) ~= R_samples
%             error('R length');
%         end
%         if (u_plan(1,2)==3)
%             plan_types(end,1) = plan_types(end,1) + 1;
%             if sum(u_plan(1:6,2) ~= bridge_plan(1:6,2)) > 0
%                 error('bridge plan mismatch');
%             end
%         else
%             plan_types(end,2) = plan_types(end,2) + 1;
%         end
%         guess_hist = [guess_hist; Pslip params.slip_fun(theta)];
%         R = mean(R);
%             
%         theta_hist = [theta_hist; theta];
%         R_hist = [R_hist; R];
%         trans_hist = [trans_hist; sum(transitions) size(transitions,1)];
%     end
%     w_hist = [w_hist; mu sigma];
%     Rmean_hist = [Rmean_hist; mean(R_hist(end-theta_samples+1:end))];
% 
%     % update policy
%     dex_start = max(1, size(theta_hist,1)-theta_samples*policy_samples+1);
%     Dtheta = theta_hist(dex_start:end, :);
%     Dw = zeros(size(Dtheta,1),1);
%     Dr = R_hist(dex_start:end, :);
%     %compute weights
%     if (params.reweight_samples)
%         for i=1:min(policy_samples, size(Dw,1)/theta_samples)
%             dex = (i-1)*theta_samples+1:(i)*theta_samples;
%             prob_sample = prod(normpdf(Dtheta(dex,:), repmat(w_hist(end-i+1, 1:thetadim), theta_samples, 1), repmat(w_hist(end-i+1, thetadim+1:end), theta_samples, 1)),2);
%             prob_current = prod(normpdf(Dtheta(dex,:), repmat(mu, theta_samples, 1), repmat(sigma, theta_samples, 1)),2);
%             Dw(dex, 1) = prob_current./prob_sample;
%             Dw(dex, 1) = Dw(dex, 1)/sum(Dw(dex, 1));
%         end
%         Dw = Dw / min(policy_samples, size(Dw,1)/theta_samples);
%     else   
%         % no weights
%         Dw = 1/size(Dr,1)*ones(size(Dr));
%         % fixed weight
%         %Dw = 1/10*ones(size(Dr));
%     end
%     
%     % add bias term?
%     
%     % dual function
%     Z = @(eta)exp((Dr-max(Dr))/eta);
%     g_fun = @(eta) eta*epsilon + max(Dr) + eta .* log(sum(Z(eta).*Dw));
%     deta_fun = @(eta) epsilon + log(sum(Z(eta).*Dw)) - sum(Z(eta).*(Dr-max(Dr)))./(eta*sum(Z(eta)));
%     deal2 = @(varargin) deal(varargin{1:nargout});
%     opt_fun = @(eta) deal2(g_fun(eta), deta_fun(eta));
%     
%     
%     eta_star = fmincon(opt_fun, [0.001], [-1], [0], [], [], [], [], [], optimset('Algorithm','interior-point', 'GradObj','on', 'MaxFunEvals', 500, 'Display', 'off'));
%     
%     Z = Z(eta_star);
%     Z_ext = repmat(Z,[1, size(Dtheta,2)]);
%     
%     %if(iter > 4)
%         mu = sum(Z_ext.*Dtheta)/sum(Z);
%         denom = ((sum(Z).^2 - sum(Z.^2))/sum(Z));
%         sigma = sqrt(sum(Z_ext.*((Dtheta-repmat(mu,[size(Dtheta,1),1])).^2))/denom);
%     %end
% 
%     if ~output_off
%         [wasted_plans eta_star mu sigma params.slip_fun(mu)]
%     end
%     
%     total_samples = total_samples + theta_samples*R_samples + wasted_plans;
%    
% end

%% display results
if ~output_off

%     slip_fun = @(theta)min(sum(theta.^2/2/length(theta)), 0.4);
%     if thetadim == 1
%         figure()
%         hold on
% 
%         z = linspace(min(theta_hist)-1, max(theta_hist)+1, 200)';
%         [m, s2, K] = model.f(model, z);
% 
%         plot_confidence(z, m, sqrt(s2));
%         plot(model.x, model.y, '+', 'MarkerSize', 12)
%         grid on
%         xlabel('input, x')
%         ylabel('output, y')
%         hold off        
%     elseif thetadim == 2
%         figure()
%         hold on
%         
%         val = zeros(100,100);
%         pred_m = val;
%         pred_s2 = val;
%         %x=linspace(min(min(theta_hist))-1,max(max(theta_hist))+1,100);
%         x=linspace(-1,1,100);
%         y=x;
%         for i=1:100
%             for j=1:100
%                 val(i,j) = slip_fun([x(i),y(j)]);
%             end
%             [m, s2, K] = model.f(model, [x(i)*ones(size(x')), x']);
%             pred_m(i, :) = m';
%             pred_s2(i, :) = s2';
%         end
%         
%         surf(x,y, pred_m);
%         scatter3(model.x(:,1),model.x(:,2),model.y); 
%         
%         hold off
%     end
    
    figure()
    hold on
    %plot(guess_hist(:,2), guess_hist(:,2), guess_hist(:,2), 0.05*ones(size(guess_hist,1)));
    %scatter(guess_hist(:,2), guess_hist(:,1));
    
    edges = 0:0.005:0.11;
    values = edges(2:end)-(edges(2)-edges(1))/2;
    guess_bars = zeros(size(values));
    guess_std = zeros(size(values));
    for i=1:length(values)
        bin_values = guess_hist( guess_hist(:,2)>=edges(i) & guess_hist(:,2)<edges(i+1), 1);
        guess_bars(i) = mean(bin_values);
        guess_std(i) = std(bin_values);
    end
    %bar(values, guess_bars);
    plot(values,values,values, 0.05*ones(size(values)));
    errorbar(values, guess_bars, guess_std);

    figure()
    plot(Rmean_hist)
    xlabel('Iteration')
    ylabel('R')
    figure()
    plot(w_hist)
    xlabel('Iteration')
    ylabel('w (mean and variance of policy parameter)')
    axis([0,100, -1, 2]);
    
    figure()
    plot(plan_types(:,1));
    
    total_samples
    
end

end


function [Tgen] = Transition_function(Pslips)
    Tgen = zeros(3,5,2);
    Ps = Pslips(1);
    Tgen(:,:,1) = [[  0   Ps   1-Ps   0    0  ];
                   [  0   0     Ps   1-Ps  0  ];
                   [  0   0     0     Ps  1-Ps]];

    Ps = Pslips(2);
    Tgen(:,:,2) = [[1-Ps  Ps    0    0    0  ];
                   [ 0   1-Ps   Ps   0    0  ];
                   [ 0    0    1-Ps  Ps   0  ]];
                
end

function [R] = R_function(Pslips)
    R = zeros(3,5,2);
    R(:,:,1) = [   [ -10   1   -5   0   0  ];
                   [  0   -1    1  -1.1   0  ];
                   [  0   0    -1   1  -10 ]];

    R(:,:,2) = R(:,:,1);
end

function [Pslip] = Slip_function(thetas)
    % same for both controllers
    Pslip = min(mean(thetas.^2/2, 2), 0.8);
end

function [mu, sigma] = reps_update(mu, sigma, D, params)
        epsilon = params.reps_epsilon;
        Dtheta = D(:,1:params.thetadim);
        Dr = D(:,params.thetadim+1);
        
        %compute weights
        Dw = zeros(size(Dtheta,1),1);
        if (params.reps_reweight)
%             for i=1:min(policy_samples, size(Dw,1)/theta_samples)
%                 dex = (i-1)*theta_samples+1:(i)*theta_samples;
%                 prob_sample = prod(normpdf(Dtheta(dex,:), repmat(w_hist(end-i+1, 1:thetadim), theta_samples, 1), repmat(w_hist(end-i+1, thetadim+1:end), theta_samples, 1)),2);
%                 prob_current = prod(normpdf(Dtheta(dex,:), repmat(mu, theta_samples, 1), repmat(sigma, theta_samples, 1)),2);
%                 Dw(dex, 1) = prob_current./prob_sample;
%                 Dw(dex, 1) = Dw(dex, 1)/sum(Dw(dex, 1));
%             end
%             Dw = Dw / min(policy_samples, size(Dw,1)/theta_samples);
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