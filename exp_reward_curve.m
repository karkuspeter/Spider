    pvals = 0:0.005:0.1;
    expRcurv = zeros(length(pvals),1);
    plan_type = expRcurv;
    realRcurv = expRcurv;
    realPcurv = expRcurv;
    for i=1:length(pvals)
        [u_plan prev_V] = plan(world, model, pvals(i));
        expRcurv(i) = prev_V(1,2);
        plan_type(i) = u_plan(1,2);
        realR = [];
        realP = [];
        for j=1:1000
            [R transitions] = execute(world,[1,2],u_plan, sqrt(2*pvals(i))); 
            realR = [realR R];
            realP = [realP sum(transitions)/length(transitions)/2];
        end
        realRcurv(i) = mean(realR);
        realPcurv(i) = mean(realP);
    end
    figure()
    hold on
    plot(pvals, expRcurv, pvals, plan_type*10);
    scatter(pvals, realRcurv);
    
    figure()
    hold on
    plot(pvals, pvals);
    scatter(pvals, realPcurv);
    