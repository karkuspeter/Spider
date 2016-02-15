    pvals = 0:0.01:0.3;
    realRcurv1 = zeros(length(pvals),1);
    realRcurv2 = zeros(length(pvals),1);
    
    prev_u = ones(size(world.Tnorm,1),1);
    plan1 = plan(world, 0, prev_u, 0);
    plan2 = plan(world, 0.2, prev_u, 0);
    
    params.R_func = @(R, theta)(R + min(0,sigmf(mean(abs(theta),2), [20 0.35])*0.3-0.5));
   
    for i=1:length(pvals)
        realR1 = [];
        realR2 = [];
        for j=1:10000
            [R transitions] = execute(world,[1,2],plan1, sqrt(2*pvals(i)), params); 
            realR1 = [realR1 R];
            [R transitions] = execute(world,[1,2],plan2, sqrt(2*pvals(i)), params); 
            realR2 = [realR2 R];
        end
        realRcurv1(i) = mean(realR1);
        realRcurv2(i) = mean(realR2);
    end
    figure()
    hold on
    plot(pvals, realRcurv1, pvals, realRcurv2, 'LineWidth', 2);
    xlabel('Pslip');
    ylabel('R');
    set(gca,'fontsize',20);
