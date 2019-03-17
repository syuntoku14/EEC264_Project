function [mse_list, mean_list] = sim_KF(r, r_, name_KF, num_epoch)
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
    r_mean = r_prior.mean * 3.75 + 0.25;
    true_model = OBKF_model(r, r_mean);

    %% r_specific_model setup
    r_specific_model = OBKF_model(r_, r_mean);

    %% Simulation
    num_k = 50;
    mse_list = zeros([1, num_k]);
    mean_list = cell(num_k, 1);

    for epoch = 1:num_epoch
        tic;
        x_k = x_0;
        y_list = {};
        if strcmp(name_KF, 'ClassicKF')
            KF = ClassicKalmanFilter(x0.mean, x0.cov, error_cov);
        elseif strcmp(name_KF, 'IBRKF')
            KF = IBRKalmanFilter(x0.mean, x0.cov, error_cov);
        elseif strcmp(name_KF, 'OBKF')
            KF = OBKalmanFilter(x0.mean, x0.cov, error_cov, true_model.ex_Q, true_model.ex_R);
        elseif strcmp(name_KF, 'MAPKF')
            KF = MAPKalmanFilter(x0.mean, x0.cov, error_cov, true_model.ex_Q, true_model.ex_R);
        end
        for k = 1:num_k
            [x_k1, y_k] = step_model(x_k, true_model);
            y_list{end+1} = y_k;
            if strcmp(name_KF, 'ClassicKF')
                KF.estimate_x_k1(y_k, r_specific_model);
                KF.compute_ex_err_cov_k1(true_model, r_specific_model);
            elseif strcmp(name_KF, 'IBRKF')
                KF.estimate_x_k1(y_k, r_specific_model);
                KF.compute_ex_err_cov_k1(true_model);
            elseif strcmp(name_KF, 'OBKF')
                mean_list{k} = [mean_list{k}, KF.estimate_x_k1(y_k, y_list, r_specific_model)];
                KF.compute_ex_err_cov_k1(true_model);
            elseif strcmp(name_KF, 'MAPKF')
                KF.estimate_x_k1(y_k, y_list, r_specific_model);
                KF.compute_ex_err_cov_k1(true_model);
            end
            mse_list(k) = mse_list(k) + trace(KF.err_cov_k);
            x_k = x_k1;
        end
        runtime = toc;
        disp(['"epoch', num2str(epoch), '": ', num2str(runtime), ' sec']); 
    end
    mse_list = mse_list ./ num_epoch; 
    mse_list = mse_list(2:end);
    mean_list = mean_list(2:end);
end


