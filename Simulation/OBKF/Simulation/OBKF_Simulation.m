% Simulation of IBRKF
% In this script, error_ellipse function is used. Please download here: 
% https://www.mathworks.com/matlabcentral/fileexchange/4705-error-ellipse
% Reference: https://ieeexplore.ieee.org/document/8246477

%% settings
addpath('../utils');
addpath('../filter');
addpath('../model');
clear all; close all;
conf_interval = 0.95;

num_epoch = 200;

%% true_model setup
% Initial State
x0.mean = [100, 10, 30, -10]';
x0.cov = [25, 0, 0, 0;
        0, 2, 0, 0;
        0, 0, 25, 0;
        0, 0, 0, 2];
x_0 = mvnrnd(x0.mean, x0.cov)';
error_cov = x0.cov;

r_prior = makedist('Uniform');
r_mean = r_prior.mean * 3.75 + 0.25;
r_max = 4.0;

r = 1.0;
r_specific_simulation(r, r_max, num_epoch, 1, 2);

% r = 3.0;
% r_specific_simulation(r, r_max, num_epoch, 3, 4);

rmpath('../utils');
rmpath('../filter');
rmpath('../model');

function r_specific_simulation(r, r_max, num_epoch, n_figure1, n_figure2)
    %% Simulation
    specific_err_list = sim_KF(r, r, 'ClassicKF', num_epoch);
    mm_err_list = sim_KF(r, r_max, 'ClassicKF', num_epoch);
    ibr_err_list = sim_KF(r, r, 'IBRKF', num_epoch);
    [obkf_err_list, mean_list] = sim_KF(r, r, 'OBKF', num_epoch);
    mapkf_err_list = sim_KF(r, r, 'MAPKF', num_epoch);
    
    fig = figure(n_figure1); hold on;
    plot(specific_err_list);
    plot(mm_err_list, 'LineStyle', '-.');
    plot(ibr_err_list);
    plot(obkf_err_list, 'LineStyle', '--');
    plot(mapkf_err_list, 'LineStyle', ':', 'Color', 'b');
    legend('Model-Specific', 'Minimax', 'IBR', 'OBKF', 'MAP');
    xlabel('k'); ylabel('MSE');
    title(['r=', int2str(r)]);
    saveas(fig, ['../result/r',int2str(r),'mse.png']);
    hold off;
    
    fig = figure(n_figure2); hold on;
    mean_list_mean = cellfun(@mean, mean_list);
    mean_list_val = cellfun(@var, mean_list);
    errorbar(mean_list_mean, mean_list_val);
    xlabel('k'); ylabel('Average Posterior Mean');
    title(['r=', int2str(r)]);
    saveas(fig, ['../result/r', int2str(r), 'mean.png']);
    hold off;
end