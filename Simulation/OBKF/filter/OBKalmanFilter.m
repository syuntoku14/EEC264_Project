classdef OBKalmanFilter < handle
    % This works only for OBKF_model
    properties
        x_0
        P_0
        x_k        
        P_k  % This is expectation according to theta
        err_cov_k
        ex_Q_y
        ex_R_y
        pi_theta
        K_k
    end
    
    methods
        function obj = OBKalmanFilter(ex_x_0, ex_P_0, err_cov_k, ex_Q, ex_R)
            obj.x_0 = ex_x_0;
            obj.P_0 = ex_P_0;
            obj.x_k = ex_x_0;
            obj.P_k = ex_P_0;
            obj.err_cov_k = err_cov_k;
            obj.ex_Q_y = ex_Q;
            obj.ex_R_y = ex_R;
            obj.pi_theta = makedist('Uniform', 'lower', 0.25, 'upper', 4);
        end
        
        function estimate_x_k1(obj, y_k, y_list, model)
            H_k = model.H_k;
            Phi_k = model.Phi_k;
            Gamma_k = model.Gamma_k;
            ex_P_k = obj.P_k;
            ex_Q_y = obj.ex_Q_y;
            ex_R_y = obj.ex_R_y;
            
            z_k = y_k - H_k*obj.x_k;
            
            K_k = ex_P_k*H_k'/(H_k*ex_P_k*H_k' + ex_R_y);
            obj.K_k = K_k;
            
            r_list = obj.MCMC(y_list, model);
            r_mean = mean(r_list);
            [ex_Q_y, ex_R_y] = obj.QR_from_r(r_mean, model); 
            
            ex_P_k1 = Phi_k*(eye(4) - K_k*H_k)*ex_P_k*Phi_k'...
                + Gamma_k*ex_Q_y*Gamma_k';
            x_k1 = Phi_k * obj.x_k + Phi_k * K_k * z_k;
            obj.ex_Q_y = ex_Q_y;
            obj.ex_R_y = ex_R_y;
            obj.x_k = x_k1;
            obj.P_k = ex_P_k1;
        end
        
        function r_list = MCMC(obj, y_list, model)
            r = obj.pi_theta.random();
            f_y_r_1 = obj.factor_graph(r, y_list, model);
            i = 1;
            num_iterations = 1000;
            r_list = [r];
            while i <= num_iterations
                r_candid = obj.pi_theta.random();
                f_y_r = obj.factor_graph(r_candid, y_list, model);
                z = min(1, f_y_r*obj.pi_theta.pdf(r_candid)/(f_y_r_1*obj.pi_theta.pdf(r)));
%                 disp(['r: ', num2str(r)]);
%                 disp(['r_candid: ', num2str(r_candid)]);
%                 disp(['f_y_r_1: ' , num2str(f_y_r_1)]);
%                 disp(['f_y_r: ' , num2str(f_y_r)]);
%                 disp(['z: ', num2str(z)]);
                if rand() < z 
                    r = r_candid;
                    f_y_r_1 = f_y_r;
                    r_list = [r_list, r];
                    %disp(['accepted: ', num2str(r)]);
                end
                % disp('-----');
                i = i+1;
            end
        end
        
        function [Q, R] = QR_from_r(obj, r, model)
            Q = model.Q;
            R = r .* eye(2);
        end
        
        function pdf_y_r = factor_graph(obj, r, y_list, model)
            M = obj.x_0;
            S = 1;
            Sigma = obj.P_0;
            Sigma_inv = inv(obj.P_0);
            i = 1;
            [Q, R] = obj.QR_from_r(r, model);
            Q_inv = inv(Q);
            R_inv = inv(R);
            while i < length(y_list) 
                W = model.H_k'*R_inv*y_list{i} + Sigma_inv*M;
                
                Lammda_inv = model.Phi_k'*Q_inv*model.Phi_k+model.H_k'*R_inv*model.H_k...
                    + Sigma_inv;
                
                Lammda = inv(Lammda_inv);
                
                new_Sigma_inv = Q_inv - Q_inv*model.Phi_k*Lammda*model.Phi_k'*Q_inv;
                
                new_Sigma = inv(new_Sigma_inv);
                
                new_M = new_Sigma*Q_inv*model.Phi_k*Lammda*(model.H_k'*R_inv*y_list{i}+Sigma_inv*M);
                
                S = S*sqrt(det(Lammda)*det(new_Sigma)/(det(Q)*det(Sigma)))...
                        * 1/(2*pi()*sqrt(det(R)))...
                        * exp((new_M'*new_Sigma_inv*new_M + W'*Lammda*W - M'*Sigma_inv*M)/2 - (y_list{i}'*R_inv*y_list{i})/2);
 
                M = new_M;
                Sigma = new_Sigma;
                Sigma_inv = new_Sigma_inv;
                i = i+1;
            end
            
            Delta_inv = model.H_k'*R_inv*model.H_k + Sigma_inv;
            Delta = inv(Delta_inv);
            G = Delta*(model.H_k'*R_inv*y_list{i} + Sigma_inv*M);
            
            pdf_y_r = S*sqrt(det(Delta)/det(Sigma))...
                *1/(2*pi()*sqrt(det(R)))...
                *exp((G'*Delta_inv*G - M'*Sigma_inv*M)/2-(y_list{i}'*R_inv*y_list{i})/2);
            pdf_y_r = double(pdf_y_r);
        end
        
        function compute_ex_err_cov_k1(obj, true_model)
            H_k = true_model.H_k;
            Phi_k = true_model.Phi_k;
            Gamma_k = true_model.Gamma_k;
            ex_R = true_model.ex_R;
            true_Q = true_model.Q; 
            true_R = true_model.R;
            ex_P_k = obj.P_k;
           
            K_k = obj.K_k;
            
            err_cov_k_1 = Phi_k*(eye(4) - K_k*H_k)*obj.err_cov_k ...
                * (eye(4) - K_k*H_k)'*Phi_k' ...
                + Gamma_k*true_Q*Gamma_k' ...
                + Phi_k*K_k*true_R*K_k'*Phi_k';
            obj.err_cov_k = err_cov_k_1;
        end
    end
end

