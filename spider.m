% load environment
world = 0;

% initialize parameters
if ~exist('output_off')
    output_off = 0;
end
R_samples=10;
theta_samples=10;
iterations=100;

w = [-0.5 2];
D = [];
polyn = 2;
%model = struct('p', zeros(polyn,1), 'f', @(x,y) glmval(x,y,'probit'));
%model = struct('p', zeros(polyn,1), 'f', @(p,x) polyval(p,x));
init_x = [-5:0.2:5]';
model = struct('p', 0, 'f', @(m, z) predictWithFullGPModel(m.p, m.x, m.y, z), 'x', init_x, 'y', rand(size(init_x))*0.1+0.3);
model.p = getFullGPModel(model.x, model.y);
%model = struct('p', 0, 'f', @(m, z) gp(m.p, @infExact, meanfunc, covfunc, likfunc, m.x, m.y, z), 'x', [], 'y', []);
%alpha = 0.004; %worked for fixed straight plan using b
alpha = 0.01;
%alpha = 0.3;

world = struct('r', -1 * ones(7,7), 'terminal', zeros(7,7));
world.r(3:5, 1) = -200;
world.r(3:5, 3:4) = -200;
world.r(7, 2) = 50;
world.terminal = (world.r ~= -1);
world.r(4, 5:7) = -60;
x0 = [1 2];

experience = [];

R_hist = [];
theta_hist = [];
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

%meanfunc = {@meanSum, {@meanLinear, @meanConst}};
%covfunc = @covSEiso;
%likfunc = @likGauss;
%hyp2.mean = [0 0]; %hyp2.cov = [0; 0]; hyp2.lik = log(0.1);
%model.p = hyp2;

%[R, t] = execute(world, x0, u_plan, 0.2)
prev_V = zeros(size(world.r));

for iter = 1:iterations
    D = [];
    for j=1:theta_samples
        % sample theta
        theta = normrnd(w(1), abs(w(2)));

        % plan 
        [u_plan prev_V] = plan(world, model, theta, prev_V);
        %u_plan = plan(world, struct('p', 0.1, 'f', @(n,m) n), 0);
        
        % execute plan, get real world experience
        R = 0;
        transitions = [];
        for i=1:R_samples
            [Ri, ti] = execute(world, x0, u_plan, theta);
            R = R + Ri/R_samples;
            transitions = [transitions; ti];
        end
        D = [D; [theta R]];
        %if theta >= -1 && theta <= 1
        experience = [experience; [theta, sum(transitions) size(transitions,1)]];
        %end
        theta_hist = [theta_hist; theta];
    end
    w_hist = [w_hist; w];
    R_hist = [R_hist; mean(D(:,2))];

    % update model
    if size(experience,1)>2*size(model.x,1)
        prob_vec = experience(:,2)./2./experience(:,3);
        theta_vec = experience(:,1);
        %f = [ones(size(theta_vec)), theta_vec, theta_vec.^2, theta_vec.^3];
        %model.p = polyfit(theta_vec, prob_vec, polyn);
        
        model.x = theta_vec;
        model.y = prob_vec;
        model.p = getFullGPModel(model.x, model.y, 500, model.p);
        
        %model.p = glmfit(theta_vec, prob_vec);
        %model.x = [model.x; theta_vec];
        %model.y = [model.y; prob_vec];
        %model.p = minimize(hyp2, @gp, -100, @infExact, meanfunc, covfunc, likfunc, model.x, model.y);
    end
    
    % update policy
    mu = w(1);
    sigma = w(2);
    grad_mu = (D(:,1) - mu)./(sigma^2);
    grad_sigma = ((D(:,1) - mu).^2 - sigma^2)./(sigma^3);
    b_mu = sum(grad_mu.^2.*D(:,2)) / sum(grad_mu.^2);
    b_sigma = sum(grad_sigma.^2.*D(:,2)) / sum(grad_sigma.^2);
    %b_mu=0;
    %b_sigma=0;
    
    delta_w = [ sigma^2 * sum(grad_mu.*(D(:,2)-b_mu));
        sigma^2 * sum(grad_sigma.*(D(:,2)-b_sigma))]'./size(D,1)*alpha;
    
    %normpdf(D(:,1), mu, sigma).*
    
    w = w + delta_w;
    if ~output_off
        w
    end
   
end

if ~output_off

    figure()
    theta=[-2:0.01:2]';
    est = model.f(model, theta);
    real = min(abs(theta.^2/2), 0.4);
    %plot(theta, est, theta, real);

    figure()
    hold on
    %prob_vec = experience(:,2)./2./experience(:,3);
    %theta_vec = experience(:,1);
    %scatter(theta_vec, prob_vec, 'filled');

    z = linspace(min(experience(:,1))-1, max(experience(:,1))+1, 100)';
    [m, s2, K] = model.f(model, z);

    f = [m+2*sqrt(s2); flipdim(m-2*sqrt(s2),1)];
    fill([z; flipdim(z,1)], f, [7 7 7]/8);
    plot(z, m, 'LineWidth', 2);
    plot(model.x, model.y, '+', 'MarkerSize', 12)
    grid on
    xlabel('input, x')
    ylabel('output, y')
    hold off

    figure()
    plot(R_hist)
    xlabel('Iteration')
    ylabel('R')
    figure()
    plot(w_hist)
    xlabel('Iteration')
    ylabel('w (mean and variance of policy parameter)')
    axis([0,100, -1, 2]);
    
end