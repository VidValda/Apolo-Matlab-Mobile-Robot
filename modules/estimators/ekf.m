function [x_est,P] = ekf(x_est_prev, u, z, beacons_xy, P, dt, params)

    v = u(1);
    w = u(2);
    theta = x_est_prev(3);

    epsilon = 1e-6; 
    if (abs(w) > epsilon)
        turn_radius = v/w;
        sin_th = sin(theta);
        cos_th = cos(theta);
        sin_th_wdt = sin(theta + w*dt);
        cos_th_wdt = cos(theta + w*dt);

        x_apriori = [
            x_est_prev(1) + turn_radius * (sin_th_wdt - sin_th);
            x_est_prev(2) - turn_radius * (cos_th_wdt - cos_th);
            theta + w*dt;
        ];        
        d_dx_dtheta = (v/w) * (cos_th_wdt - cos_th);
        d_dy_dtheta = (v/w) * (sin_th_wdt - sin_th);
        F_k = [
            1, 0, d_dx_dtheta;
            0, 1, d_dy_dtheta;
            0, 0, 1
        ];
        L_k = [
             (sin_th_wdt - sin_th)/w,  (v/(w^2))*(sin_th - sin_th_wdt) + (v/w)*dt*cos_th_wdt;
            -(cos_th_wdt - cos_th)/w, -(v/(w^2))*(cos_th - cos_th_wdt) + (v/w)*dt*sin_th_wdt;
             0,                        dt
        ];
    else
        c_th = cos(theta);
        s_th = sin(theta);
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
        L_k = [
            c_th * dt, 0;
            s_th * dt, 0;
            0,         dt
        ];
    end

    


    P = F_k * P * F_k' + L_k*params.Q*L_k';

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

        grad_scaling = 1 - (params.correction_factor / D2);

        H_k(i*2-1,:) = [-dx/D*grad_scaling, -dy/D*grad_scaling, (params.sensor_y_offset/D)*(dx*s_th-dy*c_th)*grad_scaling]; 
        H_k(i*2,:) = [dy/D2, -dx/D2, (-params.sensor_y_offset*(dx*c_th+dy*s_th)/D2)-1];        
    end


    y_k = z - z_k_pred;

    for i = 2:2:length(y_k)
        y_k(i) = atan2(sin(y_k(i)), cos(y_k(i))); 
    end

    m_beacon = size(params.R, 1);
    N_beacons = size(z, 1) / m_beacon;

    R = kron(eye(N_beacons), params.R);

    S = H_k * P * H_k' + R;
    K = P * H_k' / S;
    x_est = x_apriori + K * y_k;
    P = (eye(size(P)) - K * H_k) * P;
end
