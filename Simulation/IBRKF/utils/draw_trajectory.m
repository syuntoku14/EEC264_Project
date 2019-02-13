function [true_vec, measured_vec, estimated_vec] = draw_trajectory(num_figure, x_k, y_k, KF, conf_interval)
    color1 = [1, 0, 0];
    color2 = [0, 0, 1];
    color3 = [0, 1, 0];

    figure(num_figure); hold on;
    true_vec = draw_vector(x_k, color1);
    measured_vec = draw_vector(y_k, color2);
    estimated_vec = draw_vector(KF.x_k, color3);
    error_ellipse(KF.P_k(1:2, 1:2), KF.x_k(1:2), 'conf', conf_interval);
end

