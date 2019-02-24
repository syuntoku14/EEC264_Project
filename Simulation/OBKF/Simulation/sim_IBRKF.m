function mse_list = sim_IBRKF(r, num_epoch)
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

    r_prior = makedist('Uniform', 'lower', 0.25, 'upper', 4);
    r = r;
    r_mean = r_prior.mean;
    true_model = OBKF_model(r, r_mean);

    %% r_specific_model setup
    r_specific_model = OBKF_model(10000, r_mean);
    
    %% Simulation
    num_k = 50;
    mse_list = zeros([1, num_k]);

    for epoch = 1:num_epoch
        x_k = x_0;
        KF = IBRKalmanFilter(x0.mean, x0.cov, error_cov);
        for k = 1:num_k
            [x_k1, y_k] = step_model(x_k, true_model);
            KF.estimate_x_k1(y_k, r_specific_model);
            KF.compute_ex_err_cov_k1(true_model);
            mse_list(k) = mse_list(k) + trace(KF.err_cov_k);
            x_k = x_k1;
        end
    end
    mse_list = mse_list ./ num_epoch; 
    mse_list = mse_list(2:end);
end


