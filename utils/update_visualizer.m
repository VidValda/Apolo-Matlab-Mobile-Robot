function update_visualizer(viz, x_est, P)
    py = x_est(1); % X (Vertical)
    px = x_est(2); % Y (Horizontal)
    
    % No cambies nada aquí, MATLAB ya sabe que el eje X (tu Y horizontal) 
    % está en modo 'reverse' y pondrá el punto donde corresponde.
    addpoints(viz.h_trail, px, py);
    set(viz.h_robot, 'XData', px, 'YData', py);

    arrow_len = 0.8;
    u = arrow_len * sin(x_est(3));   % componente en Y (horizontal)
    v = arrow_len * cos(x_est(3));   % componente en X (vertical)

    set(viz.h_arrow,'XData',px,'YData',py,'UData',u,'VData',v);

    Pxy = P(1:2,1:2);
    [V,D] = eig(Pxy);
    radii = 3*sqrt(diag(D));
    ell = V*(radii .* viz.circle_pts);

    el_x = px + ell(2,:);
    el_y = py + ell(1,:);

    set(viz.h_ellipse,'XData',el_x,'YData',el_y);
    drawnow limitrate;
end
