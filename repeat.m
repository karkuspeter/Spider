rep = 5;
output_off = 1;

wh_vec = [];
Rh_vec = [];
for i=1:rep
    spider;
    wh_vec = cat(3,wh_vec,w_hist);
    Rh_vec = cat(3,Rh_vec,R_hist);
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
