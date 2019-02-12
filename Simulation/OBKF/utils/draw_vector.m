function p = draw_vector(x_k, color, varargin)

p = inputParser;
p.addRequired('x_k');
p.addRequired('color');
p.addParameter('IBRKF_flag', false);
p.parse(x_k, color, varargin{:});
inputs = p.Results;
if inputs.IBRKF_flag == true
    try    
        p = quiver(x_k(1), x_k(2), x_k(3), x_k(4), 'MaxHeadSize', 1, 'Color', color);
    catch
        p = plot(x_k(1), x_k(2), 'LineStyle', 'none', 'Marker', 'o', 'MarkerFaceColor',...
        color, 'MarkerEdgeColor', color, 'MarkerSize', 3);
    end
else
    try    
        p = quiver(x_k(1), x_k(3), x_k(2), x_k(4), 'MaxHeadSize', 1, 'Color', color);
    catch
        p = plot(x_k(1), x_k(2), 'LineStyle', 'none', 'Marker', 'o', 'MarkerFaceColor',...
        color, 'MarkerEdgeColor', color, 'MarkerSize', 3);
    end
end

