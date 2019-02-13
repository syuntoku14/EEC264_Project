classdef IBRKalmanFilter < handle
    % This works only for OBKF_model
    properties
        x_k        
        P_k  % This is expectation according to theta
        err_cov_k
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
            
            ex_P_z_k = H_k*ex_P_k*H_k';
            
            K_k = Phi_k*ex_P_k*H_k'/(ex_P_z_k + ex_R);
            
            x_k1 = Phi_k*obj.x_k + K_k*z_k;

            ex_P_k1 = (Phi_k - K_k*H_k)*ex_P_k*Phi_k'...
                + Gamma_k*ex_Q*Gamma_k';
            
            obj.x_k = x_k1;
            obj.P_k = ex_P_k1;
        end
        
        function compute_ex_err_cov_k1(obj, varargin)
           obj.err_cov_k = obj.P_k;
        end
    end
end

