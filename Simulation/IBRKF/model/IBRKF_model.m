classdef IBRKF_model < handle
    % Reference: https://ieeexplore.ieee.org/document/7869637

    % x: state vector: 4x1, [px, py, vx, vy]'
    % Phi_k: state transition matrix: 4x4
    % Gamma_k: process noise transition matrix: 4x2
    % Q: Process noise covariance matrix: 2x2

    % y: observation vector: 2x1
    % R: observation noise covariance matrix: 4x4
    % H: observation transition matrix: 2x4
    % v: observation noise: 4x1
    
    properties
        h
        Phi_k
        H_k
        Gamma_k
        theta
        theta_mean
        Q
        R
        ex_Q
        ex_R
    end
    methods
        function obj = IBRKF_model(theta, theta_mean)

        obj.h = 0.5;
        
        obj.Phi_k = [1, 0, obj.h, 0;
                        0, 1, 0, obj.h;
                        0, 0, 1, 0;
                        0, 0, 0, 1];
                    
        obj.H_k = [1, 0, 0, 0;
                     0, 1, 0, 0];
                 
        obj.Gamma_k = [obj.h^2/2, 0;
                        0, obj.h^2/2;
                        obj.h, 0;
                        0, obj.h];
        
        obj.theta = theta;
        obj.theta_mean = theta_mean;
        
        obj.Q = [4, 0;
                 0, 4];
        obj.ex_Q = [4, 0;
                    0, 4];
                
        obj.R = obj.theta .* eye(2);
        obj.ex_R = obj.theta_mean .* eye(2);
        end
        
        function set_theta(obj, theta)
            obj.theta = theta;
            obj.R = obj.theta .* eye(2);
        end
    end
end