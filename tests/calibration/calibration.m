clear; clc; close all;

robotName = 'Marvin';
dt = 0.1;

start_x = 25;
start_y = 9;
start_theta = 0;

correction_factor = 0.02;
sensor_y_offset = 0.1; 

apoloPlaceMRobot(robotName, [start_x, start_y, 0], start_theta);
apoloResetOdometry(robotName, [start_x, start_y, 0]);
apoloUpdate();

beacons_pos_map = [
    8.60,  11.00;    % ID 1  (PARED_1)
    12.00, 9.80;     % ID 2  (PARED_2)
    16.00, 7.00;     % ID 3  (PARED_3)
    20.00, 9.00;     % ID 4  (PARED_4)
    7.05,  8.55;     % ID 5  (PARED_5)
    15.30, 2.15;     % ID 6  (PARED_6)
    16.05, 15.65;    % ID 7  (PARED_7)
    24.00, 9.65;     % ID 8  (PARED_8)
    16.00, 12.00;    % ID 9  (PARED_9)
    24.15, 12.00;    % ID 10 (PARED_10)
    25.00, 15.65;    % ID 11 (PARED_11)
    29.00, 9.00;     % ID 12 (PARED_12)
    35.00, 10.00;    % ID 13 (PARED_13)
    24.65, 7.25;     % ID 14 (PARED_14)
    34.80, 7.25;     % ID 15 (PARED_15)
    35.50, 12.10;    % ID 16 (PARED_16)
    15.35, 4.95;     % ID 17 (PARED_17)
    25.35, 4.95;     % ID 18 (PARED_18)
    25.20, 2.46;     % ID 19 (PARED_19)
    35.00, 4.25;     % ID 20 (PARED_20)
    35.00, 2.35;     % ID 21 (PARED_21)
    3.75,  12.00;    % ID 22 (PARED_22)
    3.20,  18.10;    % ID 23 (PARED_23)
    36.20, 17.45;    % ID 24 (PARED_24)
    32.10, 17.65     % ID 25 (PARED_25)
];


%[v_ref (m/s), w_ref (rad/s), duración (s)]
movimientos = [
    0.4,  0.0,  10.0;
    0.2,  0.5,  3.14/0.5;
    0.4,  0.0,  10.0;
    0.2, -0.5,  3.14/0.5;
    0.3,  -0.5,  10.0;
    0.0,  0.5,  4;
];

repeticiones = 4; 
U_sequence = [];

for r = 1:repeticiones
    for k = 1:size(movimientos, 1)
        v_cmd = movimientos(k, 1);
        w_cmd = movimientos(k, 2);
        duracion = movimientos(k, 3);
        
        num_pasos_segmento = ceil(duracion / dt);
        segmento = repmat([v_cmd, w_cmd], num_pasos_segmento, 1);
        U_sequence = [U_sequence; segmento];
    end
end

total_steps = size(U_sequence, 1);

figure('Units', 'normalized', 'Position', [0.1 0.1 0.8 0.8], 'Name', 'Calibración Extendida');
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

pos_absolute_current = apoloGetLocationMRobot(robotName);
pos_absolute_past = pos_absolute_current;
pos_odometry_current = apoloGetOdometry(robotName);
pos_odometry_past = pos_odometry_current;

fprintf('Iniciando calibración... Total pasos: %d\n', total_steps);

for i = 1:total_steps
    v_actual = U_sequence(i, 1);
    w_actual = U_sequence(i, 2);
    
    apoloMoveMRobot(robotName, [v_actual, w_actual], dt);
    apoloUpdate();
    
    pos_odometry_current = apoloGetOdometry(robotName);
    pos_absolute_current = apoloGetLocationMRobot(robotName);
    
    dx_odo = pos_odometry_current(1) - pos_odometry_past(1);
    dy_odo = pos_odometry_current(2) - pos_odometry_past(2);
    dtheta_odo = wrapToPi(pos_odometry_current(3) - pos_odometry_past(3));
    
    dx_real = pos_absolute_current(1) - pos_absolute_past(1);
    dy_real = pos_absolute_current(2) - pos_absolute_past(2);
    dtheta_real = wrapToPi(pos_absolute_current(4) - pos_absolute_past(4));
    
    w_odo_meas = dtheta_odo / dt;
    w_real_meas = dtheta_real / dt;
    

    dist_cuerda_odo = sqrt(dx_odo^2 + dy_odo^2);
    if abs(dtheta_odo) > 1e-5
        factor = (dtheta_odo / 2) / sin(dtheta_odo / 2);
        v_odo_meas = (dist_cuerda_odo * factor) / dt;
    else
        v_odo_meas = dist_cuerda_odo / dt;
    end
    
    dist_cuerda_real = sqrt(dx_real^2 + dy_real^2);
    if abs(dtheta_real) > 1e-5
        factor = (dtheta_real / 2) / sin(dtheta_real / 2);
        v_real_meas = (dist_cuerda_real * factor) / dt;
    else
        v_real_meas = dist_cuerda_real / dt;
    end
    
    errors_v(i) = v_real_meas - v_odo_meas;
    errors_w(i) = w_real_meas - w_odo_meas;
    
    beacons_reading = apoloGetLaserLandMarks('LMS100');
    
    x_real_robot = pos_absolute_current(1);
    y_real_robot = pos_absolute_current(2);
    theta_real_robot = pos_absolute_current(4);
    
    x_sensor_global = x_real_robot + sensor_y_offset * cos(theta_real_robot);
    y_sensor_global = y_real_robot + sensor_y_offset * sin(theta_real_robot);
    
    for j = 1:length(beacons_reading.id)
        id = beacons_reading.id(j);
        
        if id > 0 && id <= size(beacons_pos_map, 1)
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
    
    addpoints(h1, i, errors_v(i));
    addpoints(h3, i, errors_w(i));
    
    if mod(i, 20) == 0
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

tuning_Q = 1.0; 

Q = diag([var_v, var_w]) * tuning_Q;
R = diag([var_range, var_bearing]);

fprintf('\n--- RESULTADOS DE CALIBRACIÓN ---\n');
fprintf('Q (Ruido Proceso [v, w]):\n'); disp(Q);
fprintf('R (Ruido Medida [range, bearing]):\n'); disp(R);
fprintf('Varianza v: %.6f\n', var_v);
fprintf('Varianza w: %.6f\n', var_w);

save('stats_error_motores_odometria.mat', 'Q', 'R', 'correction_factor', 'sensor_y_offset');
saveas(gcf, 'calibration_results.png');