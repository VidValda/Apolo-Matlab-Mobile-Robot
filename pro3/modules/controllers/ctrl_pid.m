function u = ctrl_pid(x_curr, ref_state, params)
    x = x_curr(1);
    y = x_curr(2);
    theta = x_curr(3);
    
    x_ref = ref_state.x;
    y_ref = ref_state.y;
    th_ref = ref_state.th;

    dx = x_ref - x;
    dy = y_ref - y;
    dtheta = th_ref - theta;
    
    dtheta = wrapToPi(dtheta);
    
    c_th = cos(theta);
    s_th = sin(theta);
    
    e_x =  c_th * dx + s_th * dy;
    e_y = -s_th * dx + c_th * dy;
    e_th = dtheta;

    v_cmd = params.Kp_v * e_x;
    
    Ky = params.Kp_w * 0.5;
    
    w_cmd = params.Kp_w * e_th + Ky * atan2(e_y, 1.0); 
    
    % v_max = 1.0; 
    % w_max = 1.5;
    % 
    % v_cmd = max(min(v_cmd, v_max), -v_max);
    % w_cmd = max(min(w_cmd, w_max), -w_max);
    % frenar cuando el error angular es grande
    if abs(e_th) > 0.6      % ~35 grados
        v_cmd = 0.3 * v_cmd;
    end
    
    % frenar cerca del punto
    d = hypot(dx, dy);
    if d < 0.8
        v_cmd = v_cmd * (d / 0.8);
    end


    u = [v_cmd; w_cmd];
end