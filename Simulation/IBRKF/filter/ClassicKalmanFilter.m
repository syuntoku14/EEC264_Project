classdef ClassicKalmanFilter < handle
    % This works only for OBKF_model
    properties
        x_k        
        P_k
        err_cov_k
    end
    
    methods
        function obj = ClassicKalmanFilter(x_0, P_0, err_cov_k)
            obj.x_k = x_0;
            obj.P_k = P_0;
            obj.err_cov_k = err_cov_k;
        end
        
        function estimate_x_k1(obj, y_k, model)
            z_k = y_k - model.H_k*obj.x_k;
            
            P_z_k = model.H_k*obj.P_k*model.H_k';
            
            K_k = model.Phi_k*obj.P_k*model.H_k'/(P_z_k + model.R);
            
            x_k1 = model.Phi_k*obj.x_k + K_k*z_k;

            P_k1 = (model.Phi_k - K_k*model.H_k)*obj.P_k*model.Phi_k'...
                + model.Gamma_k*model.Q*model.Gamma_k';
            
            obj.x_k = x_k1;
            obj.P_k = P_k1;
        end
        
        function compute_ex_err_cov_k1(obj, true_model, theta_specific_model)
            P_z_k = true_model.H_k*obj.P_k*true_model.H_k';
            
            K_k = true_model.Phi_k*obj.P_k*true_model.H_k'/(P_z_k + theta_specific_model.R);

            err_cov_k_1 = (true_model.Phi_k - K_k*true_model.H_k)*obj.err_cov_k ...
                * (true_model.Phi_k - K_k*true_model.H_k)' ...
                + true_model.Gamma_k*true_model.ex_Q*true_model.Gamma_k' ...
                + K_k*true_model.ex_R*K_k';
            obj.err_cov_k = err_cov_k_1;
        end
    end
end

