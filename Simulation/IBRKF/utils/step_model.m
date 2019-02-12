function [x_k1, y_k] = step_model(x_k, model)
    p = size(model.Q); m = size(model.R);
    u_k = mvnrnd(zeros(p(1), 1), model.Q)';
    v_k = mvnrnd(zeros(m(1), 1), model.R)';
    x_k1 = model.Phi_k * x_k + model.Gamma_k * u_k;
    y_k = model.H_k * x_k + v_k;
end