%% Model definition
% Reference paper: https://ieeexplore.ieee.org/document/8246477

% x: state vector: 4x1, [px, vx, py, vy]'
% Phi: state transition matrix: 4x4
% u: process noise vector: zero-mean 4x1
% Gamma: process noise transition matrix: 4x4

% y: observation vector: 2x1
% R: observation noise covariance matrix: 4x4
% H: observation transition matrix: 2x4
% v: observation noise: 2x1

% Initial State
x0.mean = [100, 10, 30, -10]';
x0.cov = [25, 0, 0, 0;
        0, 2, 0, 0;
        0, 0, 25, 0;
        0, 0, 0, 2;];
x_0 = mvnrnd(x0.mean, x0.cov)';

% True model definition
model.tau = 1.0;
model.Phi_k = [1, model.tau, 0, 0;
        0, 1, 0, 0;
        0, 0, 1, model.tau;
        0, 0, 0, 1];
model.H_k = [1, 0, 0, 1;
        0, 0, 1, 0];
model.Gamma_k = eye(4);
model.q = 2.0;
model.Q = model.q.*[model.tau^3/3, model.tau^2/2, 0, 0;
            model.tau^2/2, model.tau, 0, 0;
            0, 0, model.tau^3/3, model.tau^2/2;
            0, 0, model.tau^2/2, model.tau;];
model.r = 1.0;
model.R = model.r .* eye(2);
