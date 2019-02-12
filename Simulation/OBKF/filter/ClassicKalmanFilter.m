classdef ClassicKalmanFilter < handle
    % This works only for OBKF_model
    properties
        x_k        
        P_k
    end
    
    methods
        function obj = ClassicKalmanFilter(x_0, P_0)
            obj.x_k = x_0;
            obj.P_k = P_0;
        end
        
        function estimate_x_k1(obj, y_k, model_k)
            H_k = model_k.H_k;  Phi_k = model_k.Phi_k;  
            Gamma_k = model_k.Gamma_k;
            R = model_k.R;  Q = model_k.Q;
  
            z_k = y_k - H_k*obj.x_k;
            K_k = obj.P_k*H_k'/(H_k*obj.P_k*H_k' + R);
            x_k1 = Phi_k*obj.x_k + Phi_k*K_k*z_k;
            K_size = size(K_k);
            P_k1 = Phi_k*(eye(K_size(1)) - K_k*H_k)*obj.P_k*Phi_k' + Gamma_k*Q*Gamma_k';
            obj.x_k = x_k1;
            obj.P_k = P_k1;
        end
    end
end

