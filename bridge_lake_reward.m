    % world to use. comment out to use loaded one
    world = world2();
    %params.R_func = @(R, theta)(R + min(0,sigmf(mean(abs(theta),2), [20 0.35])*0.3-0.5));
    params.R_func = @(R, theta)(R + min(0,sigmf(mean(abs(theta),2), [10 0.3])*1-0.5));


    pvals = 0:0.005:0.2;
    realRcurv1 = zeros(length(pvals),1);
    realRcurv2 = zeros(length(pvals),1);
    
    prev_u = ones(size(world.Tnorm,1),1);
    plan1 = plan(world, 0, prev_u, 0);
    plan2 = plan(world, 0.2, prev_u, 0);
   
    for i=1:length(pvals)
        realR1 = [];
        realR2 = [];
        for j=1:10000
            [R transitions] = execute(world,[world.x0{:}],plan1, sqrt(2*pvals(i)), params); 
            realR1 = [realR1 R];
            [R transitions] = execute(world,[world.x0{:}],plan2, sqrt(2*pvals(i)), params); 
            realR2 = [realR2 R];
        end
        realRcurv1(i) = mean(realR1);
        realRcurv2(i) = mean(realR2);
    end
    figure()
    hold on
    plot(pvals, realRcurv1, pvals, realRcurv2, 'LineWidth', 2);
    xlabel('Pslip(\theta)');
    ylabel('R');
    ylim([-70, 50]);
    xlim([0, 0.2]);
    set(gca,'fontsize',16);
    set(gca,'color','none');
    %set(gca, 'YTick', [-40 -20 0 20 40]);
    %set(gca, 'YTickLabel', {'-40', '-20', '0', '20', '40'});
    
    
    figure()
    colorOrder = get(gca, 'ColorOrder');
    colors = [colorOrder(2,:); colorOrder(3,:)];

    hold on
    plot(pvals, realRcurv1, 'LineWidth', 2, 'Color', colors(1,:));
    plot(pvals, realRcurv2, 'LineWidth', 2, 'Color', colors(2,:));
    xlabel('Pslip(\theta)');
    ylabel('R');
    ylim([-70, 50]);
    xlim([0, 0.2]);
    set(gca,'fontsize',16);
    set(gca,'color','none');
    
    %h = legend('Bridge policy','Runaround policy', 'Location', 'best');
    h = legend({'Bridge policy','Runaround policy'}, 'Position', [0.58,0.70,0.25,0.1]);
    set(h, 'color', 'none');
    %set(gca, 'YTick', [-40 -20 0 20 40]);
    %set(gca, 'YTickLabel', {'-40', '-20', '0', '20', '40'});

    export_fig coupling.png -transparent -m5
