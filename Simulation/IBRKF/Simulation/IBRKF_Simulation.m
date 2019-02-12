% Simulation of IBRKF
% In this script, error_ellipse function is used. Please download here: 
% https://www.mathworks.com/matlabcentral/fileexchange/4705-error-ellipse
% Reference: https://ieeexplore.ieee.org/document/7869637

%% settings
addpath('../utils');
addpath('../filter');
addpath('../model');
clear all; close all;
fig = figure(1);  hold on;
conf_interval = 0.95;

%% true_model setup
% Initial State
x0.mean = [0, 0, 0, 0]';
x0.cov = 12 .* eye(4);
x_0 = mvnrnd(x0.mean, x0.cov)';
error_cov = x0.cov .* 2;

theta_prior = makedist('Uniform');
% theta_prior = makedist('Beta', 'a', 0.1, 'b', 0.9);
theta = theta_prior.random(1) * 6.0 + 0.2;
theta_mean = theta_prior.mean * 6.0 + 0.2;
true_model = IBRKF_model(theta, theta_mean);
disp(['true_theta = ', num2str(true_model.theta)]);

%% theta_specific_model setup
theta_ = theta;
theta_specific_model = IBRKF_model(theta_, theta_mean);
disp(['specific_theta = ', num2str(theta_specific_model.theta)]);

%% Simulation
color1 = [1, 0, 0];
color2 = [0, 0, 1];
color3 = [0, 1, 0];

num_epoch = 1;
num_k = 50;
err_list = zeros([1, num_k]);

for epoch = 1:num_epoch
    x_k = x_0;
    KF = ClassicKalmanFilter(x0.mean, x0.cov, error_cov);
    for k = 0:num_k - 1
        [x_k1, y_k] = step_model(x_k, true_model);
        true_vec = draw_vector(x_k, color1); 
        measured_vec = draw_vector(y_k, color3);
        estimated_vec = draw_vector(KF.x_k, color2);
        error_ellipse(KF.P_k(1:2, 1:2), KF.x_k(1:2), 'conf', conf_interval);
        KF.compute_ex_err_cov_k1(true_model, theta_specific_model);
        KF.estimate_x_k1(y_k, theta_specific_model);
        err_list(k+1) = trace(KF.err_cov_k);
        x_k = x_k1;
    end
end

legend([true_vec, estimated_vec, measured_vec], {'true x_k', 'estimated x_k', 'measured y_k'});
figure(2);
plot(err_list);


%% Simulation using different specific theta

clear all; 
fig = figure(3); hold on;

theta_ = 0.2:0.1:6.2;
classickf_mse_list = zeros([1, length(theta_)]);
minmax_mse_list = zeros([1, length(theta_)]);
for i = 1:length(theta_)
    classickf_mse = sim_classicKF_specific_theta(theta_(i));
    minmax_mse = sim_classicKF_specific_theta(6.2);
    classickf_mse_list(i) = classickf_mse(end);
    minmax_mse_list(i) = minmax_mse(end);
end
plot(theta_, classickf_mse_list);
plot(theta_, minmax_mse_list);
xlim([0.2, 6.2]);
ylim([13, 26]);
xlabel("\theta'");
ylabel("Average Steady-State MSE");
legend('Model-Specific Kalman', 'Minimax Kalman');