classdef IBRKalmanFilter < handle
    % This works only for OBKF_model
    properties
        x_k        
        P_k  % This is expectation according to theta
        err_cov_k
        K_k
    end
    
    methods
        function obj = IBRKalmanFilter(x_0, ex_P_0, err_cov_k)
            obj.x_k = x_0;
            obj.P_k = ex_P_0;
            obj.err_cov_k = err_cov_k;
        end
        
        function estimate_x_k1(obj, y_k, model)
            H_k = model.H_k;
            Phi_k = model.Phi_k;
            Gamma_k = model.Gamma_k;
            ex_R = model.ex_R;
            ex_Q = model.ex_Q;
            ex_P_k = obj.P_k;
            
            z_k = y_k - H_k*obj.x_k;
            
            K_k = ex_P_k*H_k'/(H_k*ex_P_k*H_k' + ex_R);
            
            x_k1 = Phi_k*obj.x_k + K_k*z_k;

            ex_P_k1 = Phi_k*(eye(4) - K_k*H_k)*ex_P_k*Phi_k'...
                + Gamma_k*ex_Q*Gamma_k';
            
            obj.x_k = x_k1;
            obj.P_k = ex_P_k1;
            obj.K_k = K_k;
        end
        
        function compute_ex_err_cov_k1(obj, true_model)
            H_k = true_model.H_k;
            Phi_k = true_model.Phi_k;
            Gamma_k = true_model.Gamma_k;
            ex_R = true_model.ex_R;
            true_Q = true_model.Q; 
            true_R = true_model.R;
            ex_P_k = obj.P_k;
           
            % K_k = ex_P_k*H_k'/(H_k*ex_P_k*H_k' + ex_R);
            K_k = obj.K_k;
            
            err_cov_k_1 = Phi_k*(eye(4) - K_k*H_k)*obj.err_cov_k ...
                * (eye(4) - K_k*H_k)'*Phi_k' ...
                + Gamma_k*true_Q*Gamma_k' ...
                + Phi_k*K_k*true_R*K_k'*Phi_k';
            obj.err_cov_k = err_cov_k_1;
        end
    end
end

