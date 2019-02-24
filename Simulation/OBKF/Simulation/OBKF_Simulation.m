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

num_epoch = 5;

r = 1.0;
r_specific_simulation(r, r_max, num_epoch, 1);

% r = 3.0;
% r_specific_simulation(r, r_max, num_epoch, 2);

rmpath('../utils');
rmpath('../filter');
rmpath('../model');

function r_specific_simulation(r, r_max, num_epoch, n_figure)
    %% Simulation
    specific_err_list = sim_classicKF_specific_r(r, r, num_epoch);
    mm_err_list = sim_classicKF_specific_r(r, r_max, num_epoch);
    ibr_err_list = sim_IBRKF(r, num_epoch);
    obkf_err_list = sim_OBKF(r, num_epoch);
    
    figure(n_figure); hold on;
    plot(specific_err_list);
    plot(mm_err_list, 'LineStyle', '-.');
    plot(ibr_err_list);
    plot(obkf_err_list, 'LineStyle', '--');
    legend('Model-Specific', 'Minimax', 'IBR', 'OBKF'); %, 'IBR Kalman');
    xlabel('k'); ylabel('MSE');
    title(['r=', int2str(r)]);
    hold off;
end