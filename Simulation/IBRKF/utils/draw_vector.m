function p = draw_vector(x_k, color)
try    
    p = quiver(x_k(1), x_k(2), x_k(3), x_k(4), 'MaxHeadSize', 1, 'Color', color);
catch
    p = plot(x_k(1), x_k(2), 'LineStyle', 'none', 'Marker', 'o', 'MarkerFaceColor',...
    color, 'MarkerEdgeColor', color, 'MarkerSize', 3);
end


