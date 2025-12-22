function viz = init_visualizer(map, full_path_xy, Xmax, Ymax)
    figure(1); clf; hold on; grid on;
    xlabel('Y [m]'); ylabel('X [m]');
    title('Simulación EKF (Apolo): X vertical, Y horizontal');
    
    % --- SOLUCIÓN: Voltear el mapa horizontalmente ---
    % Usamos fliplr para que el mapa coincida con la orientación de Apolo
    map_flipped = fliplr(map); 
    
    % Dibujar el mapa volteado
    imagesc([0 Ymax], [0 Xmax], map_flipped);
    
    set(gca,'YDir','normal');  % X=0 abajo
    set(gca,'XDir','reverse'); % Mantiene el 20 a la izquierda y 0 a la derecha reverse
    
    axis equal tight;


    if ~isempty(full_path_xy)
        % IMPORTANTE: Si el mapa está volteado, la coordenada Y del path 
        % debe transformarse para que caiga en el lugar correcto del dibujo.
        % La visualización ahora entiende que "0" está a la derecha.
        plot(full_path_xy(:,2), full_path_xy(:,1), 'r-', 'LineWidth', 2);
    end

    viz.h_trail   = animatedline('LineWidth', 1, 'Color', 'b');
    viz.h_robot   = plot(nan,nan,'co','MarkerFaceColor','c');
    viz.h_arrow   = quiver(nan,nan,nan,nan,'b','LineWidth',2,'MaxHeadSize',2,'AutoScale','off');
    viz.h_ellipse = plot(nan,nan,'-m','LineWidth',2);
    th = linspace(0,2*pi,40);
    viz.circle_pts = [cos(th); sin(th)];
end