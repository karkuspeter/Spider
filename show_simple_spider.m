% show
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
