hyper_params = struct('R_samples', [1 2 4 8], ...
                      'theta_samples', [6 12 18 24], ...
                      'epsilon', [0.3 0.4 0.5 0.6 0.7]);
common_params = struct('output_off', 1, 'iterations', Inf, ...
                  'total_samples', 5000);
repeat_setting = 10;

% hyper_params = struct('R_samples', [4], ...
%                       'theta_samples', [12], ...
%                       'epsilon', [0.6 0.7]);
% common_params = struct('output_off', 1, 'iterations', Inf, ...
%                   'total_samples', 1000);
% repeat_setting = 2;

% create a list of all permutations     
hp_list = [common_params]; % start with params for all executions
param_names = fieldnames(hyper_params);
for param_name = param_names(:)'
    hp_list2 = [];
    for i = 1:numel(hp_list)
        list_el = hp_list(i);
        for param_val = hyper_params.(param_name{:})
            list_el.(param_name{:}) = param_val;
            hp_list2 = [hp_list2; list_el];
        end
    end
    hp_list = hp_list2;
end

% execute each
h_stats = [];
h_linstats = [];
h_params = [];

for i = 1:numel(hp_list)
    list_el = hp_list(i);
    
    repeats = repeat_setting;
    keep_prev = 0;
    spider_func = @spider;
    show_func = @()(disp(''));
    reward_name = 'R_raw';
    
    run_struct = list_el
    
    % manually compute iterations to be always the same
    
    no_params = 1;
    repeat;
    
    % save results
    h_stats = cat(3, h_stats, rep_stats);
    h_linstats = cat(3, h_linstats, linstat_vec);
    h_params = cat(3, h_params, params);
    
    save('results/hyperparam_temp.mat');
end

% make summary
res_list = hp_list;
for i=1:numel(res_list)
    res_list(i).total_samples = h_stats(i).mean_linstat.total_samples;
    res_list(i).Rcumm_mean = h_stats(i).Rcumm_mean;
    res_list(i).Rcumm_std = h_stats(i).Rcumm_std;
end

%save
save(strcat('results/hyper-', datestr(now,'dd-mm-yyyy-HH-MM'), '.mat'));

% show specific result
indices = [1 2];
for ind = indices
    rep_stats = h_stats(:,:,ind);
    linstat_vec = h_linstats(:,:,ind);
    params = h_params(:,:,ind);
    params
    show_spider()
end

no_params = 0;