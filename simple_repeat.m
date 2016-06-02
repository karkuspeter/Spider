if ~exist('no_params') | no_params == 0
    repeats = 10;
    run_struct = struct('output_off',1,'method',2);
end

stat_vec = [];
linstat_vec = [];
cumm_rew_vec = [];

for i=1:repeats
    
    [stat, linstat] = simple_spider(run_struct);
    cumm_rew = mean(linstat.reward);

    stat_vec = [stat_vec; stat];
    linstat_vec = [linstat_vec; linstat];
    cumm_rew_vec(:,:,end+1) = cumm_rew;
end

% compue mean
fields = fieldnames(linstat);
for i=1:numel(fields)
    linstat.(fields{i}) = linstat_vec(1).(fields{i});
    for j=2:length(linstat_vec)
        linstat.(fields{i}) = linstat.(fields{i}) + linstat_vec(j).(fields{i});
    end
    linstat.(fields{i}) = linstat.(fields{i}) / length(linstat_vec);
end


% show
Rcumm_mean = mean(cumm_rew_vec,3)
Rcumm_std = std(cumm_rew_vec,0,3)

figure()
subplot(1, 2, 1);
plot([linstat.Ps_avg(:,1) linstat.Ps_est(:,1)]);
xlabel('iter');
legend('Ps_{avg}', 'Ps_{est}');
subplot(1, 2, 2);
plot([linstat.Ps_avg(:,2) linstat.Ps_est(:,2)]);
xlabel('iter');
legend('Ps_{avg}', 'Ps_{est}');

figure()
plot(linstat.policy + repmat([0 0.1 0.2], size(linstat.policy,1),1));
xlabel('iter');
legend('policy(1)','policy(2)','policy(3)');

% figure()
% hold on
% plot_confidence((1:size(R_mean,1))', R_mean, R_std);
% xlabel('Iteration')
% ylabel('Rcumm')