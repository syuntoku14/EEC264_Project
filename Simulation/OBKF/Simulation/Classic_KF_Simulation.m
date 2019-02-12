% Simulation of Kalman_Filter
% In this script, error_ellipse function is used. Please download here: 
% https://www.mathworks.com/matlabcentral/fileexchange/4705-error-ellipse
% Reference paper: https://ieeexplore.ieee.org/document/8246477

%% settings
addpath('../utils');
addpath('../filter');
addpath('../model');
clear; close all;
fig = figure(1);  hold on;

%% model setup
OBKF_model;  % model for OBKF paper

%% Simulation
color1 = [1, 0, 0];
color2 = [0, 0, 1];
color3 = [0, 1, 0];

num_epoch = 1;
for epoch = 1:num_epoch
    x_k = x_0;
    KF = ClassicKalmanFilter(x0.mean, x0.cov);
    for k = 0:49
        [x_k1, y_k] = step_model(x_k, model);
        p1 = draw_vector(x_k, color1);  p2 = draw_vector(KF.x_k, color2);  p3 = draw_vector(y_k, color3);
        KF.estimate_x_k1(y_k, model);
        x_k = x_k1;
    end
end

legend([p1, p2, p3], {'true x_k', 'estimated x_k', 'measured y_k'});
% fig = figure(2);
% plot(0:49, mse_classic ./ num_epoch);

%% Functions
function mse_ = mse(P_k)
    mse_ = mean((x1 - x2).^2);
end