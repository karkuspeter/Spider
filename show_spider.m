
figure()
hold on
plot_confidence((1:size(rep_stats.mean_linstat.R_mean,1))', rep_stats.mean_linstat.R_mean, rep_stats.std_linstat.R_mean);
xlabel('Iteration')
ylabel('R')


figure()
plot_confidence((1:size(rep_stats.mean_linstat.theta_mu,1))', rep_stats.mean_linstat.theta_mu, rep_stats.std_linstat.theta_mu, 0, 0, params.slip_fun);
xlabel('Iteration')
ylabel('slip(theta_mu) (mean of slip function)')

figure()
plot_confidence((1:size(rep_stats.mean_linstat.plan_type,1))', rep_stats.mean_linstat.plan_type(:,1), rep_stats.std_linstat.plan_type(:,1));
xlabel('Iteration')
ylabel('Bridge plan used');

% figure()
% plot_confidence((1:size(w_mean,1))', w_mean(:,2), w_std(:,2));
% xlabel('Iteration')
% ylabel('w (variance of policy parameter)')