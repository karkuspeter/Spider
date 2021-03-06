% load environment

if ~exist('minimize')
    run ../gpml-matlab-v3.6-2015-07-07/startup.m;
    addpath ../SPGP_dist;
end






% initialize parameters
params = struct('R_samples', 0, 'theta_samples', 0, 'iterations', 0, ...
                'policy_samples', 0, 'thetadim', 0, 'epsilon', 0, ...
                'mu', 0, 'sigma', 0, 'R_dependency', 0, ...
                'plan_off', 0, 'reweight_samples', 0, ...
                'R_func', 0, 'slip_fun', 0, 'trans_cheat', 0);

            
            
            
            

%theta_reward_func = @(theta)min(0,sqrt(mean(abs(theta/2), 2))-0.5);
%%theta_reward_func = @(theta)min(0,sigmf(sqrt(mean(abs(theta.^2),2)), [20 0.35])*0.3-0.5);

if ~exist('output_off')
    output_off = 0;
end
%if ~exist('plan_off')
%    plan_off = 0;
%end
%if ~exist('reweight_samples')
%    reweight_samples = 1;
%end
params.R_samples=8;
params.theta_samples=100;
params.iterations=150;
params.policy_samples=6;
params.thetadim = 8;
params.epsilon = 0.60;
params.trans_cheat = 6;
%sparseM = 500; % number of pseudo-inputs
%GPoffset = 0.3; 

%params.mu = [-0.5 -1 0.5];
params.mu = -0.5 * ones(1, params.thetadim);
params.sigma = 1 * ones(1, params.thetadim);

params.plan_off = 0;
params.reweight_samples = 0;
params.R_dependency = 1;

params.slip_fun = @(theta)min(mean(theta.^2/2, 2), 0.4);
if (params.R_dependency)
    params.R_func = @(R, theta)(R + min(0,sigmf(mean(abs(theta),2), [20 0.35])*0.3-0.5));
else
    params.R_func = @(R, theta)(R);
end

R_samples = params.R_samples; theta_samples=params.theta_samples;
iterations = params.iterations; policy_samples = params.policy_samples;
thetadim = params.thetadim; epsilon = params.epsilon;
mu = params.mu; sigma = params.sigma;

init_p = 0.2;

model = struct('p', 0, 'f', @(m, z) m.p);
model.p = init_p;

if ~exist('world')
    world = world1();
end
x0 = [1 2];

R_hist = [];
Rmean_hist = [];
theta_hist = [];
trans_hist = [];
w_hist = [];
guess_hist = [];
prev_V = zeros(size(world.r));
total_samples = 0;

%compare planners
% p1 = plan(world, 0.2);
% p2 = plan2(world, struct('p', 0.2, 'f', @(m,t) m.p), 0.2);
% p2(p2==0) = 1;
% p1==p2

[dummy, init_plan] = plan(world, init_p, 0, 0);
bridge_plan = plan(world, 0, 0, 0);
plan_types = [];

for iter = 1:iterations
    D = [];
    wasted_plans = 0;
    plan_types = [plan_types; [0 0]];
    
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
                [u_plan, plan_raw] = plan(world, Pslip, prev_plan, R_est);
            end

            if ~isequal(plan_raw, prev_plan)
                wasted_plans = wasted_plans + i;
                i = 1;
                R = [];
                R_model = [];
                %transitions = [];
            end
            % execute plan, get real world experience
            [Ri, ti, Rnomi] = execute(world, x0, u_plan, theta, params);
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
        if (u_plan(1,2)==3)
            plan_types(end,1) = plan_types(end,1) + 1;
            if sum(u_plan(1:6,2) ~= bridge_plan(1:6,2)) > 0
                error('bridge plan mismatch');
            end
        else
            plan_types(end,2) = plan_types(end,2) + 1;
        end
        guess_hist = [guess_hist; Pslip params.slip_fun(theta)];
        R = mean(R);
            
        theta_hist = [theta_hist; theta];
        R_hist = [R_hist; R];
        trans_hist = [trans_hist; sum(transitions) size(transitions,1)];
    end
    w_hist = [w_hist; mu sigma];
    Rmean_hist = [Rmean_hist; mean(R_hist(end-theta_samples+1:end))];

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
    
    total_samples = total_samples + theta_samples*R_samples + wasted_plans;
   
end

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