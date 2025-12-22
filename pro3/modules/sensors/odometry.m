function [u_odometry, curr_odo] = odometry(robotName, last_odo, dt)
    curr_odo = apoloGetOdometry(robotName);
    
    dx = curr_odo(1) - last_odo(1);
    dy = curr_odo(2) - last_odo(2);
    d_theta = curr_odo(3) - last_odo(3);
    d_theta = atan2(sin(d_theta), cos(d_theta));
    
    chord = sqrt(dx^2 + dy^2);
    
    motion_angle = atan2(dy, dx);
    heading_diff = abs(atan2(sin(motion_angle - last_odo(3)), cos(motion_angle - last_odo(3))));
    
    direction = 1;
    if heading_diff > pi/2
        direction = -1;
    end
    
    if abs(d_theta) > 1e-6
        arc_length = chord * (d_theta / 2) / sin(d_theta / 2);
    else
        arc_length = chord;
    end
    
    v = direction * arc_length / dt;
    w = d_theta / dt;
    
    u_odometry = [v; w];
    last_odo = curr_odo;
end