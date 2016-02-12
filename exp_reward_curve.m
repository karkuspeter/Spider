    pvals = 0:0.005:0.1;
    expRcurv = zeros(length(pvals),1);
    plan_type = expRcurv;
    plan_type2 = expRcurv;
    realRcurv = expRcurv;
    realPcurv = expRcurv;
    time1 = 0;
    time3 = 0;
    time4 = 0;
    time2 = 0;
    prev_u = ones(size(world.Tnorm,1),1);
    for i=1:length(pvals)
        tic;
        [u_plan2 prev_V] = plan2(world, model, pvals(i));
        time2 = time2 + toc;
        
        tic;
        [u_plan prev_u prev_V temp] = plan(world, pvals(i), prev_u);
        time1 = time1 + toc;

        expRcurv(i) = prev_V(1,2);
        plan_type(i) = u_plan(1,2);
        plan_type2(i) = u_plan2(1,2);
        realR = [];
        realP = [];
        tic
        for j=1:100
            [R transitions] = execute(world,[1,2],u_plan, sqrt(2*pvals(i))); 
            realR = [realR R];
            realP = [realP sum(transitions)/length(transitions)/2];
        end
        time3 = time3 + toc;
        realRcurv(i) = mean(realR);
        realPcurv(i) = mean(realP);
    end
    figure()
    hold on
    plot(pvals, expRcurv, pvals, plan_type*10, pvals, plan_type2*10);
    scatter(pvals, realRcurv);
    
    figure()
    hold on
    plot(pvals, pvals);
    scatter(pvals, realPcurv);
    
    %1D theta
    figure()
    theta_reward_func = @(theta)min(0,sigmf(mean(abs(theta),2), [20 0.35])*0.3-0.5);
    slip_fun = @(theta)min(mean(theta.^2/2, 2), 0.4);
    theta = -1.5:0.01:1.5;
    plot(theta, theta_reward_func(theta'), theta, slip_fun(theta'));
    
    %10D theta
    figure()
    theta_reward_func = @(theta)min(0,sigmf(sqrt(mean(abs(theta.^2),2)), [20 0.35])*0.3-0.5);
    slip_fun = @(theta)min(mean(theta.^2/2, 2), 0.4);
    theta = -1.5:0.01:1.5;
    thetaD = zeros(size(theta,2),3);
    thetaD(:,1) = theta';
    thetaD(:,2) = theta';
    thetaD(:,3) = theta';
    plot(theta, theta_reward_func(thetaD), theta, slip_fun(thetaD), theta, zeros(size(theta)));
    
    time1 
    time2
    