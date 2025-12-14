function update_visualizer(viz, x_est, P)
    
    px = x_est(1) * viz.scale + viz.ox;
    py = viz.oy - x_est(2) * viz.scale;

    addpoints(viz.h_trail, px, py);

    arrow_len = 50; 
    
    u_vec = arrow_len * cos(x_est(3));
    v_vec = -arrow_len * sin(x_est(3));

    set(viz.h_arrow, 'XData', px, 'YData', py, 'UData', u_vec, 'VData', v_vec);

    P_xy = P(1:2, 1:2);
    
    [V, D] = eig(P_xy); 
    
    radii = 3 * sqrt(diag(D)); 
    
    ellipse_pts_m = V * (radii .* viz.circle_pts);
    
    el_x = (ellipse_pts_m(1,:) + x_est(1)) * viz.scale + viz.ox;
    el_y = viz.oy - (ellipse_pts_m(2,:) + x_est(2)) * viz.scale;
    
    set(viz.h_ellipse, 'XData', el_x, 'YData', el_y);

    drawnow limitrate;
end