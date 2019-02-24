%% settings
addpath('../utils');
addpath('../filter');
addpath('../model');
addpath('../Simulation');

close all; clear all;

%% true_model setup
% Initial State
x0.mean = [100, 10, 30, -10]';
x0.cov = [25, 0, 0, 0;
        0, 2, 0, 0;
        0, 0, 25, 0;
        0, 0, 0, 2];
x_0 = mvnrnd(x0.mean, x0.cov)';
error_cov = x0.cov;
conf_interval = 0.95;

r_prior = makedist('Uniform', 'lower', 0.25, 'upper', 4);
r = 1.0;
r_mean = r_prior.mean;
true_model = OBKF_model(r, r_mean);

%% r_specific_model setup
r_specific_model = OBKF_model(10000, r_mean);

%% Simulation
num_epoch = 1;
num_k = 50;
mse_list = zeros([1, num_k]);
y_list = {};
results = [];
pause on;

for epoch = 1:num_epoch
    x_k = x_0;
    KF = OBKalmanFilter(x0.mean, x0.cov, error_cov, true_model.ex_Q, true_model.ex_R);
    IBRKF = IBRKalmanFilter(x0.mean, x0.cov, error_cov);
    for k = 1:num_k
        [x_k1, y_k] = step_model(x_k, true_model);
        if k > 40
            [true_vec, measured_vec, estimated_vec] = draw_trajectory(1, x_k, y_k, KF, conf_interval);
            [IBRtrue_vec, IBRmeasured_vec, IBRestimated_vec] = draw_trajectory(2, x_k, y_k, IBRKF, conf_interval); 
        end
        y_list{end+1} = y_k;
        KF.estimate_x_k1(y_k, y_list, r_specific_model);
        IBRKF.estimate_x_k1(y_k, r_specific_model);
%         r_list = KF.MCMC(y_list, r_specific_model);
%         results = [results, mean(r_list)];
%         plot(results);
%         pause(0.01);
        x_k = x_k1;
        step_model(x_k, true_model);
    end
end

figure(1);
legend([true_vec, estimated_vec, measured_vec], {'true x_k', 'estimated x_k', 'measured y_k'});
title('Optimal Bayesian Kalman Filter');

figure(2);
title('IBR Kalman Filter');
legend([IBRtrue_vec, IBRestimated_vec, IBRmeasured_vec], {'true x_k', 'estimated x_k', 'measured y_k'});
