classdef ClassicKalmanFilter < handle
    % This works only for OBKF_model
    properties
        x_k        
        P_k
        err_cov_k
        K_k
    end
    
    methods
        function obj = ClassicKalmanFilter(x_0, P_0, err_cov_k)
            obj.x_k = x_0;
            obj.P_k = P_0;
            obj.err_cov_k = err_cov_k;
        end
        
        function estimate_x_k1(obj, y_k, model)
            H_k = model.H_k;
            Phi_k = model.Phi_k;
            R = model.R;
            Gamma_k = model.Gamma_k;
            Q = model.Q;
            
            z_k = y_k - H_k*obj.x_k;
            
            K_k = obj.P_k*H_k'/(H_k*obj.P_k*H_k' + R);
            obj.K_k = K_k;
            
            x_k1 = Phi_k*obj.x_k + Phi_k*K_k*z_k;

            P_k1 = Phi_k*( eye(4) - K_k*H_k)*obj.P_k*Phi_k'...
                + Gamma_k*Q*Gamma_k';
            
            obj.x_k = x_k1;
            obj.P_k = P_k1;
        end
        
        function compute_ex_err_cov_k1(obj, true_model, theta_specific_model)
            H_k = true_model.H_k;
            Phi_k = true_model.Phi_k;
            Gamma_k = true_model.Gamma_k;
            R = theta_specific_model.R;
            true_Q = true_model.Q; 
            true_R = true_model.R;
           
            K_k = obj.K_k;
            
            err_cov_k_1 = Phi_k*(eye(4) - K_k*H_k)*obj.err_cov_k ...
                * (eye(4) - K_k*H_k)'*Phi_k' ...
                + Gamma_k*true_Q*Gamma_k' ...
                + Phi_k*K_k*true_R*K_k'*Phi_k';
            obj.err_cov_k = err_cov_k_1;
        end
    end
end

