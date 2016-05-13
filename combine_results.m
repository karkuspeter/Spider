
integrated = 'integrated50.mat';
bridge = 'bridge50.mat';
lake = 'lake50.mat';
colorOrder = get(gca, 'ColorOrder');

colors = [colorOrder(3,:); colorOrder(2,:); colorOrder(1,:)];

figure()
hold on

load(lake);
r_lake = cumm_reward;
h_lake = plot_confidence((1:size(R_mean,1))', R_mean, R_std, colors(1,:), colors(1,:));

load(bridge);
r_bridge = cumm_reward;
h_bridge = plot_confidence((1:size(R_mean,1))', R_mean, R_std, colors(2,:), colors(2,:));


load(integrated);
r_int = cumm_reward;
h_int = plot_confidence((1:size(R_mean,1))', R_mean, R_std, colors(3,:), colors(3,:) );

ylim([-60,60]);
xlim([0,40]);
xlabel('Iteration')
ylabel('R')

h = legend([h_int, h_bridge, h_lake], { ...
    sprintf('Integrated approach'), ...
    sprintf('Fixed bridge policy'), ...
    sprintf('Fixed runaround policy') ...
    }, 'Location', 'southeast');
set(h, 'color', 'none');
set(gca,'color','none')
set(gca,'fontsize',16);

export_fig results.png -transparent -m5

%legend({'p1' 'p3'});


% 
% 
% figure()
% plot_confidence((1:size(w_mean,1))', w_mean(:,1:end/2), w_std(:,1:end/2), params.slip_fun);
% xlabel('Iteration')
% ylabel('w (mean of slip function)')
% 
% % figure()
% % plot_confidence((1:size(w_mean,1))', w_mean(:,2), w_std(:,2));
% % xlabel('Iteration')
% % ylabel('w (variance of policy parameter)')
% 
% [sum(Rh_vec(end-1,1,:) < 5), size(Rh_vec,3)]
% 
% cumm_reward = mean(sum(Rh_vec(:, :, :), 1),3)
% 
% if ~exist('cumm_rhist')
%     cumm_rhist = [];
% end
% cumm_rhist = [cumm_rhist; cumm_reward];
% output_off = 0;
