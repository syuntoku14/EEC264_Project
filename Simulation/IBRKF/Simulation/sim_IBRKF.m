function mse_list = sim_IBRKF(theta_)
    %% settings
    addpath('../utils');
    addpath('../filter');
    addpath('../model');
    
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

    %% theta_specific_model setup
    theta_specific_model = IBRKF_model(theta_, theta_mean);

    %% Simulation
    num_epoch = 1;
    num_k = 50;
    mse_list = zeros([1, num_k]);

    for epoch = 1:num_epoch
        x_k = x_0;
        KF = IBRKalmanFilter(x0.mean, x0.cov, error_cov);
        for k = 0:num_k - 1
            [x_k1, y_k] = step_model(x_k, true_model);
            KF.compute_ex_err_cov_k1();
            KF.estimate_x_k1(y_k, theta_specific_model);
            mse_list(k+1) = trace(KF.err_cov_k);
            x_k = x_k1;
        end
    end
end


