function ekf_sine_path
    clear; clc; close all;

    robotName = 'Marvin';
    laserName = 'LMS100';
    
    path_x = linspace(-3.5, 3.5, 100);
    path_y = 2 * sin(1.5 * path_x);
    path_points = [path_x; path_y];
    current_target_idx = 2;

    Map = containers.Map('KeyType','double','ValueType','any');
    Map(1) = [-3.9, 3.9];
    Map(2) = [3.9, 3.9];

    start_y = 2 * sin(1.5 * -3.5);
    mu = [-3.5; start_y; 0]; 
    P = eye(3) * 0.1; 
    
    Q = diag([0.05, 0.05, 0.01].^2); 
    R = diag([0.1^2, 0.05^2]);
    
    dt = 0.1;
    
    figure('Color', 'white');
    hold on; axis equal; grid on;
    xlabel('X [m]'); ylabel('Y [m]');
    xlim([-5 5]); ylim([-5 5]);
    
    plot(path_x, path_y, 'k--', 'LineWidth', 1);
    
    hOdom = plot(0,0, 'g--', 'LineWidth', 1); 
    hEst  = plot(0,0, 'b-', 'LineWidth', 2);
    hEllipse = plot(0,0, 'r-');
    
    plot([-3.9 3.9], [3.9 3.9], 'ks', 'MarkerFaceColor', 'y');
    
    apoloPlaceMRobot(robotName, [-3.5, start_y, 0.2], 0);
    apoloResetOdometry(robotName, [-3.5, start_y, 0]);
    apoloUpdate();
    
    history_mu = [];
    history_odom = []; 
    
    while current_target_idx <= length(path_points)
        
        target_x = path_points(1, current_target_idx);
        target_y = path_points(2, current_target_idx);
        
        dx = target_x - mu(1);
        dy = target_y - mu(2);
        dist_to_target = sqrt(dx^2 + dy^2);
        
        if dist_to_target < 0.3
            current_target_idx = current_target_idx + 1;
            continue;
        end
        
        target_theta = atan2(dy, dx);
        error_theta = target_theta - mu(3);
        error_theta = atan2(sin(error_theta), cos(error_theta)); 
        
        w = 1.5 * error_theta;
        v = 0.01;
        
        if abs(error_theta) > 0.5
            v = 0.01;
        end
        
        apoloMoveMRobot(robotName, [v, w], dt); 
        
        theta = mu(3);
        G = eye(3);
        G(1,3) = -v * dt * sin(theta);
        G(2,3) =  v * dt * cos(theta);
        
        mu(1) = mu(1) + v * dt * cos(theta);
        mu(2) = mu(2) + v * dt * sin(theta);
        mu(3) = mu(3) + w * dt;
        mu(3) = atan2(sin(mu(3)), cos(mu(3)));
        
        P = G * P * G' + Q;
        
        obs = apoloGetLaserLandMarks(laserName); 
        
        if ~isempty(obs.id)
            for i = 1:length(obs.id)
                lid = obs.id(i);
                z_meas = [obs.distance(i); obs.angle(i)];
                
                if isKey(Map, lid)
                    lm_pos = Map(lid);
                    lx = lm_pos(1);
                    ly = lm_pos(2);
                    
                    dx_lm = lx - mu(1);
                    dy_lm = ly - mu(2);
                    q = dx_lm^2 + dy_lm^2;
                    dist_pred = sqrt(q);
                    angle_pred = atan2(dy_lm, dx_lm) - mu(3);
                    angle_pred = atan2(sin(angle_pred), cos(angle_pred));
                    
                    z_hat = [dist_pred; angle_pred];
                    
                    H = [-(dx_lm/dist_pred), -(dy_lm/dist_pred), 0;
                          (dy_lm/q),        -(dx_lm/q),        -1];
                      
                    S = H * P * H' + R;
                    K = P * H' / S;
                    
                    y_res = z_meas - z_hat;
                    y_res(2) = atan2(sin(y_res(2)), cos(y_res(2)));
                    
                    mu = mu + K * y_res;
                    mu(3) = atan2(sin(mu(3)), cos(mu(3)));
                    P = (eye(3) - K * H) * P;
                end
            end
        end
        
        apoloUpdate(); 
        
        true_pose = apoloGetOdometry(robotName); 
        
        history_mu = [history_mu, mu];
        history_odom = [history_odom, true_pose(1:2)']; 
        
        set(hOdom, 'XData', history_odom(1,:), 'YData', history_odom(2,:)); 
        set(hEst,  'XData', history_mu(1,:), 'YData', history_mu(2,:));
        
        [eigenvec, eigenval] = eig(P(1:2,1:2));
        if eigenval(1,1) >= eigenval(2,2)
            ang = atan2(eigenvec(2,1), eigenvec(1,1));
            sc1 = sqrt(eigenval(1,1)); 
            sc2 = sqrt(eigenval(2,2));
        else
            ang = atan2(eigenvec(2,2), eigenvec(1,2));
            sc1 = sqrt(eigenval(2,2));
            sc2 = sqrt(eigenval(1,1));
        end
        
        t_ell = linspace(0, 2*pi, 50);
        x_ell = 2 * sc1 * cos(t_ell);
        y_ell = 2 * sc2 * sin(t_ell);
        R_ell = [cos(ang) -sin(ang); sin(ang) cos(ang)];
        coords = R_ell * [x_ell; y_ell] + mu(1:2);
        
        set(hEllipse, 'XData', coords(1,:), 'YData', coords(2,:));
        
        drawnow limitrate;
    end
end