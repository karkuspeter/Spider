% load environment
world = 0;

if ~exist('minimize')
    run ../gpml-matlab-v3.6-2015-07-07/startup.m;
    addpath ../SPGP_dist;
end

% initialize parameters
if ~exist('output_off')
    output_off = 0;
end
if ~exist('plan_off')
    plan_off = 0;
end
if ~exist('reweight_samples')
    reweight_samples = 1;
end
R_samples=4;
theta_samples=25;
iterations=60;
policy_samples=4;
thetadim = 3;
epsilon = 0.85;
sparseM = 500; % number of pseudo-inputs
GPoffset = 0.3;


mu = -0.5 * ones(1, thetadim);
sigma = 2 * ones(1, thetadim);
init_x = repmat([-5:0.2:5]', [1,thetadim]);
for i=1:thetadim
    init_x(:,i) = init_x(randperm(size(init_x,1)),i);
end

%model = struct('p', zeros(polyn,1), 'f', @(x,y) glmval(x,y,'probit'));
%model = struct('p', zeros(polyn,1), 'f', @(p,x) polyval(p,x));

%model = struct('p', 0, 'f', @(m, z) predictWithFullGPModel(m.p, m.x, m.y, z), 'x', init_x, 'y', rand(size(init_x))*0.1+0.3);
%model.p = getFullGPModel(model.x, model.y);
model = struct('p', 0, 'f', @(m, z) spgp_pred(m.y,m.x,m.xpseudo,z,m.p)+GPoffset, 'x', init_x, 'y', rand(size(init_x,1),1)*0.1+0.3-GPoffset, 'xpseudo', 0);
[model.p model.xpseudo] = optimizeGP(model.x, model.y, sparseM);

%model = struct('p', 0, 'f', @(m, z) gp(m.p, @infExact, meanfunc, covfunc, likfunc, m.x, m.y, z), 'x', [], 'y', []);
%alpha = 0.004; %worked for fixed straight plan using b
%alpha = 0.01;
%alpha = 0.3;

world = struct('r', -1 * ones(7,7), 'terminal', zeros(7,7));
world.r(3:5, 1) = -200;
world.r(3:5, 3:4) = -200;
world.r(7, 2) = 50;
world.terminal = (world.r ~= -1);
world.r(4, 5:7) = -60;
x0 = [1 2];

R_hist = [];
Rmean_hist = [];
theta_hist = [];
trans_hist = [];
w_hist = [];

%try planner
% temp = [];
% x=[0:0.01:0.2]';
% for i=x'
%     u_plan = plan(world, struct('p', i, 'f', @(n,m) n), 0);
%     [R, t] = execute(world, x0, u_plan, 0);
%     temp = [temp R];
% end
% figure();
% plot(x', temp);
% y=temp';

%[R, t] = execute(world, x0, u_plan, 0.2)
prev_V = zeros(size(world.r));

for iter = 1:iterations
    D = [];
    for j=1:theta_samples
        % sample theta
        theta = normrnd(mu, sigma);

        % plan 
        if(plan_off)
            u_plan = plan(world, struct('p', 0, 'f', @(m,t) m.p), 0);
        else
            Pslip = max(0,model.f(model, theta));
            [u_plan prev_V] = plan(world, model, Pslip);%, prev_V);
        end
        
        % execute plan, get real world experience
        R = 0;
        transitions = [];
        for i=1:R_samples
            [Ri, ti] = execute(world, x0, u_plan, theta);
            R = R + Ri/R_samples;
            transitions = [transitions; ti];
        end
        %D = [D; [theta R]];
        %if theta >= -1 && theta <= 1
        %experience = [experience; [theta, sum(transitions) size(transitions,1)]];
        %end
        theta_hist = [theta_hist; theta];
        R_hist = [R_hist; R];
        trans_hist = [trans_hist; sum(transitions) size(transitions,1)];
    end
    w_hist = [w_hist; mu sigma];
    Rmean_hist = [Rmean_hist; mean(R_hist(end-theta_samples+1:end))];

    % update model
     if (~plan_off) && (size(trans_hist,1)>2*size(model.x,1) || size(model.x,1) == size(init_x,1))
         prob_vec = trans_hist(:,1)./2./trans_hist(:,2);
         
         model.x = theta_hist;
         model.y = prob_vec-GPoffset;
         [model.p model.xpseudo] = optimizeGP(model.x, model.y, sparseM);
         %model.p = getFullGPModel(model.x, model.y);
     end
    
    % update policy
    dex_start = max(1, size(theta_hist,1)-theta_samples*policy_samples+1);
    Dtheta = theta_hist(dex_start:end, :);
    Dw = zeros(size(Dtheta,1),1);
    Dr = R_hist(dex_start:end, :);
    %compute weights
    if (reweight_samples)
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
    % planner sometimes stucks in 10000 iterations, why?
    
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
 

    if ~output_off
        [eta_star mu sigma]
    end
   
end

if ~output_off

    slip_fun = @(theta)min(sum(theta.^2/2/length(theta)), 0.4);
    
    if thetadim == 1
        figure()
        hold on
        %prob_vec = experience(:,2)./2./experience(:,3);
        %theta_vec = experience(:,1);
        %scatter(theta_vec, prob_vec, 'filled');

        z = linspace(min(theta_hist)-1, max(theta_hist)+1, 200)';
        [m, s2, K] = model.f(model, z);

        plot_confidence(z, m, sqrt(s2));
        plot(model.x, model.y, '+', 'MarkerSize', 12)
        grid on
        xlabel('input, x')
        ylabel('output, y')
        hold off        
    elseif thetadim == 2
        figure()
        hold on
        
        val = zeros(100,100);
        pred_m = val;
        pred_s2 = val;
        %x=linspace(min(min(theta_hist))-1,max(max(theta_hist))+1,100);
        x=linspace(-1,1,100);
        y=x;
        for i=1:100
            for j=1:100
                val(i,j) = slip_fun([x(i),y(j)]);
            end
            [m, s2, K] = model.f(model, [x(i)*ones(size(x')), x']);
            pred_m(i, :) = m';
            pred_s2(i, :) = s2';
        end
        
        surf(x,y, pred_m);
        scatter3(model.x(:,1),model.x(:,2),model.y); 
        
        hold off
    end
    
    figure()
    plot(Rmean_hist)
    xlabel('Iteration')
    ylabel('R')
    figure()
    plot(w_hist)
    xlabel('Iteration')
    ylabel('w (mean and variance of policy parameter)')
    axis([0,100, -1, 2]);
    
end