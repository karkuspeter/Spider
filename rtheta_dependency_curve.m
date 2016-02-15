    % R theta dependency effect
    thetavals = -0.2:0.002:1;
    world1 = world1();
    plan_type1 = zeros(length(thetavals),1);
    plan_type2 = plan_type1;
    expRcurv1 = plan_type1;
    expRcurv2 = plan_type1;
    realRcurv = plan_type1;
    thetadim = ones(1,3);
        
    params2 = params;
    params.R_func = @(R, theta)(R);
    params2.R_func = @(R, theta)(R + min(0,sigmf(mean(abs(theta),2), [20 0.35])*0.3-0.5));
    
    prev_u = ones(size(world.Tnorm,1),1);
    for i=1:length(thetavals)
        pval = params.slip_fun(thetadim*thetavals(i));
        [u_plan1 prev_u prev_V temp] = plan(world1, pval, prev_u, params.R_func(0, thetadim*thetavals(i)));
        expRcurv1(i) = prev_V(1,2);
        plan_type1(i) = u_plan1(1,2); 

        [u_plan2 prev_u prev_V temp] = plan(world1, pval, prev_u, params2.R_func(0, thetadim*thetavals(i)));
        expRcurv2(i) = prev_V(1,2);
        plan_type2(i) = u_plan2(1,2);    
        
        realR = [];
        for j=1:100
            [R transitions] = execute(world1,[1,2],u_plan2, thetadim*thetavals(i), params2); 
            realR = [realR R];
            %realP = [realP sum(transitions)/length(transitions)/2];
        end
        realRcurv(i) = mean(realR);
        %realPcurv(i) = mean(realP);
    end
    
    figure()
    hold on
    plot(thetavals, expRcurv1, thetavals, expRcurv2, thetavals, plan_type1*10, thetavals, plan_type2*10);
    scatter(thetavals, realRcurv);
