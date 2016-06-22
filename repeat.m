if ~exist('no_params') || no_params == 0
    repeats = 10;
    keep_prev = 0;
    func = @simple_spider;
    run_struct = struct('output_off',1,'method',2);
end

if (~keep_prev) || ~exist(stat_vec)
    stat_vec = [];
    linstat_vec = [];
    cumm_rew_vec = [];
end

for i=1:repeats
    disp(sprintf('%d / %d', i, repeats));
    
    [stat, linstat] = func(run_struct);
    cumm_rew = mean(linstat.reward);

    stat_vec = [stat_vec; stat];
    linstat_vec = [linstat_vec; linstat];
    cumm_rew_vec(:,:,end+1) = cumm_rew;
end


% show
Rcumm_mean = mean(cumm_rew_vec,3)
Rcumm_std = std(cumm_rew_vec,0,3)

figure()
subplot(1, 2, 1);
plot([mean_linstat.Ps_avg(:,1) mean_linstat.Ps_est(:,1)]);
xlabel('iter');
legend('Ps_{avg}', 'Ps_{est}');
subplot(1, 2, 2);
plot([mean_linstat.Ps_avg(:,2) mean_linstat.Ps_est(:,2)]);
xlabel('iter');
legend('Ps_{avg}', 'Ps_{est}');

figure()
plot(mean_linstat.policy + repmat([0 0.1 0.2], size(mean_linstat.policy,1),1));
xlabel('iter');
legend('policy(1)','policy(2)','policy(3)');


% if ~keep_prev || ~exist('wh_vec') || ~exist('Rh_vec')
%     wh_vec = [];
%     Rh_vec = [];
% end
% for irep=1:rep
%     spider;
%     wh_vec = cat(3,wh_vec,w_hist);
%     Rh_vec = cat(3,Rh_vec,Rmean_hist);
%     [irep, sum(Rh_vec(end-1,1,:) < 10)]
% end
% 
% w_mean = mean(wh_vec,3);
% R_mean = mean(Rh_vec,3);
% w_std = std(wh_vec,0,3);
% R_std = std(Rh_vec,0,3);
% 
% figure()
% hold on
% plot_confidence((1:size(R_mean,1))', R_mean, R_std);
% xlabel('Iteration')
% ylabel('R')
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
