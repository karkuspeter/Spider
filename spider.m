function [stats, linstat] = spider(input_params)

if ~exist('input_params')
    input_params = struct();
end
if ~exist('minimize')
    run ../../gpml-matlab-v3.6-2015-07-07/startup.m;
    addpath ../../SPGP_dist;
    addpath ../../altmany-export_fig-b894ce6;
end

% initialize parameters
params = struct('R_samples', 0, 'theta_samples', 0, 'iterations', 0, ...
                'policy_samples', 0, 'thetadim', 0, 'epsilon', 0, ...
                'mu', 0, 'sigma', 0, 'R_dependency', 0, ...
                'plan_off', 0, 'reweight_samples', 0, ...
                'R_func', 0, 'slip_fun', 0, 'trans_cheat', 0, ...
                'use_mdp_for_eval', 0);


%theta_reward_func = @(theta)min(0,sqrt(mean(abs(theta/2), 2))-0.5);
%%theta_reward_func = @(t heta)min(0,sigmf(sqrt(mean(abs(theta.^2),2)), [20 0.35])*0.3-0.5);

if isfield(input_params,'output_off')
    output_off = input_params.output_off;
else
    output_off = 0;
end

% define default param values

params.R_samples=2;
params.theta_samples=12;
params.iterations=40;
params.policy_samples=1;
params.thetadim = 3;
params.epsilon = 0.50;
params.trans_cheat = 1;
params.use_mdp_for_eval = 1;

params.mu = -0.5 * ones(1, params.thetadim);
params.sigma = 1 * ones(1, params.thetadim);

params.plan_off = 0;
params.reweight_samples = 0;
params.R_dependency = 1;

params.slip_fun = @(theta)min(mean(theta.^2/2, 2), 0.4);
if (params.R_dependency)
    %params.R_func = @(R, theta)(R + min(0,sigmf(mean(abs(theta),2), [20 0.35])*0.3-0.5));
    params.R_func = @(R, theta)(R + min(0,sigmf(mean(abs(theta),2), [10 0.3])*1-0.5));
else
    params.R_func = @(R, theta)(R);
end

%overright params that are set in input_params
fields = fieldnames(params);
for i=1:numel(fields)
    if isfield(input_params, fields{i})
        params.(fields{i}) = input_params.(fields{i});
    end
end

% statistics

linstat = struct('R', [], 'R_raw', [], 'R_real', [], 'R_exp', [], 'R_mean', [], ...
                 'theta_mu', [], 'theta_sigma', [], ...
                 'theta', [], 'trans', [], 'total_samples', 0, 'plan_type', [], ...
                 'Ps_est', [], 'Ps_real', [] );
                 %'policy', [], 'reward', [], ...
                 %'Ps_est', [], 'Ps_real', [], 'Ps_avg', []);

R_hist = [];
theta_hist = [];

R_samples = params.R_samples; theta_samples=params.theta_samples;
iterations = params.iterations; policy_samples = params.policy_samples;
thetadim = params.thetadim; epsilon = params.epsilon;
mu = params.mu; sigma = params.sigma;

init_p = 0.2;

model = struct('p', 0, 'f', @(m, z) m.p);
model.p = init_p;

world = world2();
x0 = world.x0;


prev_V = zeros(size(world.r));


%compare planners
% p1 = plan(world, 0.2);
% p2 = plan2(world, struct('p', 0.2, 'f', @(m,t) m.p), 0.2);
% p2(p2==0) = 1;
% p1==p2

[dummy, init_plan] = plan(world, init_p, 0, 0);
%bridge_plan = plan(world, 0.3, 0, 0);
bridge_plan = plan(world, 0, 0, 0);

for iter = 1:iterations
    D = [];
    wasted_plans = 0;
    linstat.plan_type = [linstat.plan_type; [0 0]];
    
    for j=1:theta_samples
        % sample theta
        theta = normrnd(mu, sigma);

        R = [];
        R_model = [];
        transitions = [];
        Pslip = init_p;
        R_est = 0;
        prev_plan = init_plan;
        while size(R,1) < R_samples
            % plan 
            if(params.plan_off)
                u_plan = bridge_plan;
                plan_raw = init_plan;
            else
                %Pslip = max(0,model.f(model, theta));
                [u_plan, plan_raw, V] = plan(world, Pslip, prev_plan, R_est);
            end

            if ~params.use_mdp_for_eval && ~isequal(plan_raw, prev_plan)
                wasted_plans = wasted_plans + i;
                i = 1;
                R = [];
                R_model = [];
                %transitions = [];
            end
            % execute plan, get real world experience
            [Ri, ti, Rnomi] = execute(world, cell2mat(x0), u_plan, theta, params);
            R = [R; Ri];
            R_model = [R_model; Rnomi];
            transitions = [transitions; ti];
            
            prev_plan = plan_raw;
            Pslip = sum(transitions)/size(transitions,1)/2;
            R_est = (Ri-Rnomi)*params.trans_cheat/length(ti);
            % now estimate is perfect, but afterwards should use history
%             if abs(R_est - params.R_func(0, theta)) > 0.0001
%                 R_est
%             end
            %should be weighted mean with length of transitions

        end
        if length(R) ~= R_samples
            error('R length');
        end
        if (u_plan(x0{:})==3)
            linstat.plan_type(end,1) = linstat.plan_type(end,1) + 1;
            %note this is world specific!
            if sum(u_plan(2:7,2) ~= bridge_plan(2:7,2)) > 0
                error('bridge plan mismatch');
            end
        else
            linstat.plan_type(end,2) = linstat.plan_type(end,2) + 1;
        end
        Ps_real = params.slip_fun(theta);
        
        linstat.Ps_est = [linstat.Ps_est; Pslip];
        linstat.Ps_real = [linstat.Ps_real; Ps_real];
        
        R = mean(R);
            
        % to check correctness compute real best plan (for actual theta)
        Pslip_real = params.slip_fun(theta);
        [~, ~, V_real] = plan(world, Pslip_real, prev_plan, R_est);
        
        linstat.R_raw = [linstat.R_raw; R];
        linstat.R_exp = [linstat.R_exp; V(x0{:})];
        linstat.R_real = [linstat.R_real; V_real(x0{:})];
        
        % overwright R that is a feedback for policy search
        if (params.use_mdp_for_eval)
            R = V(x0{:});
        end
        R_hist = [R_hist; R];
        theta_hist = [theta_hist; theta];
        
        linstat.R = [linstat.R; R];
        linstat.theta = [linstat.theta; theta];
        linstat.trans = [linstat.trans; sum(transitions) size(transitions,1)];
    end
    
    linstat.theta_mu = [linstat.theta_mu; mu];
    linstat.theta_sigma = [linstat.theta_sigma; sigma];
    linstat.R_mean = [linstat.R_mean; mean(linstat.R(end-theta_samples+1:end))];

    % update policy
    dex_start = max(1, size(theta_hist,1)-theta_samples*policy_samples+1);
    Dtheta = theta_hist(dex_start:end, :);
    Dw = zeros(size(Dtheta,1),1);
    Dr = R_hist(dex_start:end, :);
    %compute weights
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
    
    % add bias term?
    
    % dual function
    Z = @(eta)exp((Dr-max(Dr))/eta);
    g_fun = @(eta) eta*epsilon + max(Dr) + eta .* log(sum(Z(eta).*Dw));
    deta_fun = @(eta) epsilon + log(sum(Z(eta).*Dw)) - sum(Z(eta).*(Dr-max(Dr)))./(eta*sum(Z(eta)));
    deal2 = @(varargin) deal(varargin{1:nargout});
    opt_fun = @(eta) deal2(g_fun(eta), deta_fun(eta));
    
    
    eta_star = fmincon(opt_fun, [0.001], [-1], [0], [], [], [], [], [], optimset('Algorithm','interior-point', 'GradObj','on', 'MaxFunEvals', 500, 'Display', 'off'));
    
    Z = Z(eta_star);
    Z_ext = repmat(Z,[1, size(Dtheta,2)]);
    
    %if(iter > 4)
        mu = sum(Z_ext.*Dtheta)/sum(Z);
        denom = ((sum(Z).^2 - sum(Z.^2))/sum(Z));
        sigma = sqrt(sum(Z_ext.*((Dtheta-repmat(mu,[size(Dtheta,1),1])).^2))/denom);
    %end

    if ~output_off
        [wasted_plans eta_star mu sigma params.slip_fun(mu)]
    end
    
    linstat.total_samples = linstat.total_samples + theta_samples*R_samples + wasted_plans;
   
end

if ~output_off

    figure()
    hold on
    %plot(guess_hist(:,2), guess_hist(:,2), guess_hist(:,2), 0.05*ones(size(guess_hist,1)));
    %scatter(guess_hist(:,2), guess_hist(:,1));
    
    edges = 0:0.005:0.11;
    values = edges(2:end)-(edges(2)-edges(1))/2;
    guess_bars = zeros(size(values));
    guess_std = zeros(size(values));
    for i=1:length(values)
        bin_values = linstat.Ps_est(linstat.Ps_real>=edges(i) & linstat.Ps_real<edges(i+1));
        guess_bars(i) = mean(bin_values);
        guess_std(i) = std(bin_values);
    end
    plot(values,values,values, 0.05*ones(size(values)));
    errorbar(values, guess_bars, guess_std);

    figure()
    plot(linstat.R_mean)
    xlabel('Iteration')
    ylabel('R')
    figure()
    plot([linstat.theta_mu, linstat.theta_sigma])
    xlabel('Iteration')
    ylabel('w (mean and variance of policy parameter)')
    axis([0,100, -1, 2]);
    
    figure()
    plot(linstat.plan_type(:,1));
    
    linstat.total_samples
    
    
    plot(linstat.R_exp - linstat.R_real)
    figure()
    plot(1:length(linstat.R_real), linstat.R_raw - linstat.R_real, ...
         1:length(linstat.R_real), linstat.R_exp - linstat.R_real)
    xlabel('Iteration')
    ylabel('DeltaR')
    
end