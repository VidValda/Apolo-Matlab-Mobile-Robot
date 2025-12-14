function viz = init_visualizer(map, full_path_xy)
    scale = 100; 
    ox = size(map, 2)/2; 
    oy = size(map, 1)/2; 

    pixel_path = full_path_xy * scale; 
    pixel_path(:,1) = pixel_path(:,1) + ox;
    pixel_path(:,2) = oy - pixel_path(:,2); 

    figure(1); clf;
    imshow(map); hold on;
    title('Simulación EKF: Robot, Incertidumbre y Trayectoria');
    
    plot(pixel_path(:,1), pixel_path(:,2), 'r-', 'LineWidth', 2);

    h_trail = animatedline('Color', 'c', 'LineWidth', 1);
    
    h_ellipse = plot(nan, nan, '-m', 'LineWidth', 2);
    h_arrow = quiver(nan, nan, nan, nan, 'Color', 'b', 'LineWidth', 2, 'MaxHeadSize', 5, 'AutoScale', 'off');

    hold off;

    viz.h_arrow = h_arrow;
    viz.h_trail = h_trail;
    viz.h_ellipse = h_ellipse;
    
    viz.scale = scale;
    viz.ox = ox;
    viz.oy = oy;
    
    theta_circle = linspace(0, 2*pi, 30);
    viz.circle_pts = [cos(theta_circle); sin(theta_circle)];
end