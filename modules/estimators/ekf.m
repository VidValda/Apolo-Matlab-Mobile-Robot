function [x_est,P] = ekf(x_est_prev, u, z, beacons_xy, P, dt, params)

    v = u(1);
    w = u(2);

    epsilon = 1e-6; 
    if (abs(w) > epsilon)
        turn_radius = v/w;
        x_apriori = [
            x_est_prev(1) + turn_radius * (sin(x_est_prev(3)+w*dt)-sin(x_est_prev(3)));
            x_est_prev(2) - turn_radius * (cos(x_est_prev(3)+w*dt)-cos(x_est_prev(3)));
            x_est_prev(3) + w*dt;
        ];        
        d_dx_dtheta = (v/w) * (cos(x_est_prev(3) + w*dt) - cos(x_est_prev(3)));
        d_dy_dtheta = (v/w) * (sin(x_est_prev(3) + w*dt) - sin(x_est_prev(3)));
        F_k = [
            1, 0, d_dx_dtheta;
            0, 1, d_dy_dtheta;
            0, 0, 1
        ];
    else
        x_apriori = [
            x_est_prev(1) + v*cos(x_est_prev(3))*dt;
            x_est_prev(2) + v*sin(x_est_prev(3))*dt;
            x_est_prev(3); 
        ];
        F_k = [
            1, 0, -v * dt * sin( x_est_prev(3));
            0, 1,  v * dt * cos( x_est_prev(3));
            0, 0,  1
        ];
    end


    P = F_k * P * F_k' + params.Q;

    x_a = x_apriori(1);
    y_a = x_apriori(2);
    theta_a = x_apriori(3);

    c_th = cos(theta_a);
    s_th = sin(theta_a);
    
    z_k_pred = zeros(size(z));
    H_k = zeros([size(z,1),size(x_est_prev,1)]);
    
    for i = 1:(length(z_k_pred)/2)
        bx = beacons_xy(i,1);
        by = beacons_xy(i,2);

        x_sensor = x_a + params.sensor_y_offset * c_th;
        y_sensor = y_a +  params.sensor_y_offset * s_th;
        
        dx = bx - x_sensor;
        dy = by - y_sensor; 
        D2 = dx^2 + dy^2;
        D = sqrt(D2);
        z_k_pred(i*2-1) = D;
        z_k_pred(i*2) = wrapTo2Pi(atan2(dy, dx) - theta_a);

        H_k(i*2-1,:) = [-dx/D, -dy/D, (params.sensor_y_offset/D)*(dx*s_th-dy*c_th)]; 
        H_k(i*2,:) = [dy/D2, -dx/D2, (-params.sensor_y_offset*(dx*c_th+dy*s_th)/D2)-1];        
    end


    y_k = z-z_k_pred;

    m_beacon = size(params.R, 1);
    N_beacons = size(z, 1) / m_beacon;

    R = kron(eye(N_beacons), params.R);

    S = H_k * P * H_k' + R;
    K = P * H_k' / S;
    x_est = x_apriori + K * y_k;
    P = (eye(size(P)) - K * H_k) * P;
end
