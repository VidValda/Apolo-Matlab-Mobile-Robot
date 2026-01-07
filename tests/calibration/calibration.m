clear; clc; close all;

robotName = 'Marvin';
dt = 0.1;

start_x = 0;
start_y = 2;
correction_factor = 0.02;
sensor_y_offset = 0.1; 

apoloPlaceMRobot(robotName, [start_x, start_y, 0], 0);
apoloResetOdometry(robotName, [start_x, start_y, 0]);
apoloUpdate();

beacon_1_pos = [-3.9, 3.9];
beacon_2_pos = [3.9, 3.9];
beacons_pos_map = [beacon_1_pos; beacon_2_pos];

pos_absolute_current = apoloGetLocationMRobot(robotName);
pos_absolute_past = pos_absolute_current;
pos_odometry_current = apoloGetOdometry(robotName);
pos_odometry_past = pos_odometry_current;

radio = 2;
vueltas = 4;
distanciatotal = 2*pi*radio;
velocidadL = 0.2;
numerodepasos = ceil(distanciatotal/(velocidadL*dt));
velocidadA = -2*pi/(numerodepasos*dt);
total_steps = ceil(numerodepasos * vueltas);

figure('Units', 'normalized', 'Position', [0.1 0.1 0.8 0.8]);
ax1 = subplot(4,2,1); title('Error v (m/s)'); grid on; h1 = animatedline(ax1, 'Color', 'r');
ax2 = subplot(4,2,2); title('Histogram Error v'); grid on; h2 = histogram('FaceColor', 'b');
ax3 = subplot(4,2,3); title('Error w (rad/s)'); grid on; h3 = animatedline(ax3, 'Color', 'k');
ax4 = subplot(4,2,4); title('Histogram Error w'); grid on; h4 = histogram('FaceColor', 'b');
ax5 = subplot(4,2,5); title('Error Bearing (rad)'); grid on; h5 = animatedline(ax5, 'Color', 'r');
ax6 = subplot(4,2,6); title('Histogram Error Bearing'); grid on; h6 = histogram('FaceColor', 'b');
ax7 = subplot(4,2,7); title('Error Range (m)'); grid on; h7 = animatedline(ax7, 'Color', 'k');
ax8 = subplot(4,2,8); title('Histogram Error Range'); grid on; h8 = histogram('FaceColor', 'b');

errors_v = zeros(total_steps, 1);
errors_w = zeros(total_steps, 1);
errors_range = []; 
errors_bearing = [];

for i = 1:total_steps
    apoloMoveMRobot(robotName, [velocidadL, velocidadA], dt);
    apoloUpdate();
    
    pos_odometry_current = apoloGetOdometry(robotName);
    pos_absolute_current = apoloGetLocationMRobot(robotName);
    
    % --- Locomotion Error Analysis ---
    dx_odo = pos_odometry_current(1) - pos_odometry_past(1);
    dy_odo = pos_odometry_current(2) - pos_odometry_past(2);
    dtheta_odo = wrapToPi(pos_odometry_current(3) - pos_odometry_past(3));
    
    w_odo = dtheta_odo / dt;
    dist_cuerda_odo = sqrt(dx_odo^2 + dy_odo^2);

    if abs(dtheta_odo) > 1e-5
        factor_correccion = (dtheta_odo / 2) / sin(dtheta_odo / 2);
        dist_arco_odo = dist_cuerda_odo * factor_correccion;
        
        v_odo = dist_arco_odo / dt;
    else
        v_odo = dist_cuerda_odo / dt;
    end
    
    dx_real = pos_absolute_current(1) - pos_absolute_past(1);
    dy_real = pos_absolute_current(2) - pos_absolute_past(2);
    dtheta_real = wrapToPi(pos_absolute_current(4) - pos_absolute_past(4));
    
    w_real = dtheta_real / dt;
    dist_cuerda = sqrt(dx_real^2 + dy_real^2);

    if abs(dtheta_real) > 1e-5
        factor_correccion = (dtheta_real / 2) / sin(dtheta_real / 2);
        dist_arco = dist_cuerda * factor_correccion;
        
        v_real = dist_arco / dt;
    else
        v_real = dist_cuerda / dt;
    end
    
    err_v = v_real - v_odo;
    err_w = w_real - w_odo;
    
    errors_v(i) = err_v;
    errors_w(i) = err_w;
    
    % --- Sensor Error Analysis ---
    beacons_reading = apoloGetLaserLandMarks('LMS100');
    
    x_real_robot = pos_absolute_current(1);
    y_real_robot = pos_absolute_current(2);
    theta_real_robot = pos_absolute_current(4);
    
    x_sensor_global = x_real_robot + sensor_y_offset * cos(theta_real_robot);
    y_sensor_global = y_real_robot + sensor_y_offset * sin(theta_real_robot);
    
    for j = 1:length(beacons_reading.id)
        id = beacons_reading.id(j);
        
        if id <= size(beacons_pos_map, 1)
            b_x = beacons_pos_map(id, 1);
            b_y = beacons_pos_map(id, 2);
            
            dx = b_x - x_sensor_global;
            dy = b_y - y_sensor_global;
            
            range_true = sqrt(dx^2 + dy^2);
            bearing_true = wrapToPi(atan2(dy, dx) - theta_real_robot);
            
            dist_reading = beacons_reading.distance(j);
            bias_estimado = correction_factor / dist_reading;
            range_meas = dist_reading - bias_estimado;
            
            bearing_meas = wrapToPi(beacons_reading.angle(j));
            
            err_r = range_true - range_meas;
            err_b = wrapToPi(bearing_true - bearing_meas);
            
            errors_range = [errors_range; err_r];
            errors_bearing = [errors_bearing; err_b];
            
            addpoints(h5, i, err_b);
            addpoints(h7, i, err_r);
        end
    end
    
    pos_odometry_past = pos_odometry_current;
    pos_absolute_past = pos_absolute_current;
    
    addpoints(h1, i, err_v);
    addpoints(h3, i, err_w);
    
    if mod(i, 10) == 0
        h2.Data = errors_v(1:i);
        h4.Data = errors_w(1:i);
        h6.Data = errors_bearing;
        h8.Data = errors_range;
        drawnow limitrate;
    end
end

var_v = var(errors_v);
var_w = var(errors_w);
var_range = var(errors_range);
var_bearing = var(errors_bearing);

Q = diag([var_v, var_w]);
R = diag([var_range, var_bearing]);

fprintf('EKF Parameters:\n');
fprintf('Q (Process Noise Covariance): \n'); disp(Q);
fprintf('R (Measurement Noise Covariance): \n'); disp(R);

save('stats_error_motores_odometria.mat', 'Q', 'R', 'correction_factor', 'sensor_y_offset');
saveas(gcf, 'error_analysis_figure.png');