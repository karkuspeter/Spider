rep = 10;
output_off = 1;
keep_prev = 0;

if ~keep_prev || ~exist('wh_vec') || ~exits('Rh_vec')
    wh_vec = [];
    Rh_vec = [];
end
for irep=1:rep
    spider;
    wh_vec = cat(3,wh_vec,w_hist);
    Rh_vec = cat(3,Rh_vec,Rmean_hist);
    irep, sum(Rh_vec(end-1,1,:) < 10)di
end

w_mean = mean(wh_vec,3);
R_mean = mean(Rh_vec,3);
w_std = std(wh_vec,0,3);
R_std = std(Rh_vec,0,3);

figure()
hold on
plot_confidence((1:size(R_mean,1))', R_mean, R_std);
xlabel('Iteration')
ylabel('R')

figure()
plot_confidence((1:size(w_mean,1))', w_mean(:,1), w_std(:,1));
xlabel('Iteration')
ylabel('w (mean of policy parameter)')

figure()
plot_confidence((1:size(w_mean,1))', w_mean(:,2), w_std(:,2));
xlabel('Iteration')
ylabel('w (variance of policy parameter)')

[sum(Rh_vec(end-1,1,:) < 10), size(Rh_vec,3)]

