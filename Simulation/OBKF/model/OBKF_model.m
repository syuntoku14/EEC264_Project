classdef OBKF_model < handle
    % Reference: https://ieeexplore.ieee.org/document/8246477
    % x: state vector: 4x1, [px, vx, py, vy]'
    % Phi_k: state transition matrix: 4x4
    % Gamma_k: process noise transition matrix: 4x4
    % Q: Process noise covariance matrix: 4x4

    % y: observation vector: 2x1
    % R: observation noise covariance matrix: 2x2
    % H: observation transition matrix: 2x4
    % v: observation noise: 2x1
    
    properties
        tau
        Phi_k
        H_k
        Gamma_k
        r
        r_mean
        q
        Q
        R
        ex_Q
        ex_R
    end
    methods
        function obj = OBKF_model(r, r_mean)

        obj.tau = 1.0;
        
        obj.r = r;
        obj.r_mean = r_mean;
        
        obj.q = 2.0;
        
        obj.Phi_k = [1, obj.tau, 0, 0;
                    0, 1, 0, 0;
                    0, 0, 1, obj.tau;
                    0, 0, 0, 1];
                    
        obj.H_k = [1, 0, 0, 1;
                     0, 0, 1, 0];
                 
        obj.Gamma_k = eye(4);
        
        obj.Q = obj.q .* [obj.tau^3/3, obj.tau^2/2, 0, 0;
                        obj.tau^2/2, obj.tau, 0, 0;
                        0, 0, obj.tau^3/3, obj.tau^2/2;
                        0, 0, obj.tau^2/2, obj.tau];
        obj.ex_Q = obj.Q;
                    
        obj.R = obj.r .* eye(2);
        obj.ex_R = obj.r_mean .* eye(2);
        end
        
        function set_r(obj, r)
            obj.r = r;
            obj.R = obj.r .* eye(2);
        end
    end
end