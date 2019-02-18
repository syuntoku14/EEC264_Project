function mse_list = sim_classicKF_specific_r(r, r_)
    %% settings
    addpath('../utils');
    addpath('../filter');
    addpath('../model');
    
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
    r = r;
    r_mean = r_prior.mean * 3.75 + 0.25;
    true_model = OBKF_model(r, r_mean);

    %% r_specific_model setup
    r_ = r_;  % minimax
    r_specific_model = OBKF_model(r_, r_mean);

    %% Simulation
    num_epoch = 1;
    num_k = 50;
    mse_list = zeros([1, num_k]);

    for epoch = 1:num_epoch
        x_k = x_0;
        KF = ClassicKalmanFilter(x0.mean, x0.cov, error_cov);
        for k = 0:num_k - 1
            [x_k1, y_k] = step_model(x_k, true_model);
            KF.compute_ex_err_cov_k1(true_model, r_specific_model);
            KF.estimate_x_k1(y_k, r_specific_model);
            mse_list(k+1) = trace(KF.err_cov_k);
            x_k = x_k1;
        end
    end
    mse_list = mse_list(2:end);
end


