clear; clc; close all;

robotName = 'Marvin';
dt = 0.01;

start_x = 0;
start_y = 2;

pos_absolute_current = apoloGetLocationMRobot(robotName);
pos_absolute_past = pos_absolute_current;
pos_odometry_current = [pos_absolute_current(1),pos_absolute_current(2),pos_absolute_current(4)];
pos_odometry_past = pos_odometry_current;



apoloPlaceMRobot(robotName, [start_x, start_y, 0], 0);
apoloResetOdometry(robotName, [start_x, start_y, 0]);
apoloUpdate();

radio = 2;
vueltas = 2;
distanciatotal = 2*pi*radio;
velocidadL = 0.1;
numerodepasos = ceil(distanciatotal/(velocidadL*dt));
velocidadA = -2*pi/(numerodepasos*dt);

figure;
ax1 = subplot(2,2,1);
title('Error d');
grid on;
h1 = animatedline(ax1, 'Color', 'r');

ax2 = subplot(2,2,2);
title('Histogram error d');
grid on;
h2 = histogram('FaceColor', 'b', 'BinMethod', 'auto');

ax3 = subplot(2,2,3); 
title('Error Theta');
grid on;
h3 = animatedline(ax3, 'Color', 'k');

ax4 = subplot(2,2,4);
title('Histogram error Theta');
grid on;
h4 = histogram('FaceColor', 'b', 'BinMethod', 'auto');

total_steps = numerodepasos * vueltas;
errores_d = [];
errores_a = [];

for i = 0:total_steps
    apoloMoveMRobot(robotName, [velocidadL,velocidadA], dt);
    apoloUpdate();
    
    pos_odometry_current = apoloGetOdometry(robotName);
    x_odometry_current = pos_odometry_current(1);
    y_odometry_current = pos_odometry_current(2);
    theta_odometry_current = pos_odometry_current(3);
    x_odometry_past = pos_odometry_past(1);
    y_odometry_past = pos_odometry_past(2);
    theta_odometry_past = pos_odometry_past(3);

    odometry_d = sqrt((x_odometry_current - x_odometry_past)^2 + (y_odometry_current - y_odometry_past)^2);
    diff_odo = theta_odometry_current - theta_odometry_past;
    odometry_a = mod(diff_odo + pi, 2*pi) - pi;

    pos_absolute_current = apoloGetLocationMRobot(robotName);
    x_real_current = pos_absolute_current(1);
    y_real_current = pos_absolute_current(2);
    theta_real_current = pos_absolute_current(4);
    x_real_past = pos_absolute_past(1);
    y_real_past = pos_absolute_past(2);
    theta_real_past = pos_absolute_past(4);

    odometry_d_real = sqrt((x_real_current - x_real_past)^2 + (y_real_current - y_real_past)^2);
    diff_real = theta_real_current - theta_real_past;
    odometry_a_real = mod(diff_real + pi, 2*pi) - pi;

    error_d = odometry_d_real - odometry_d;
    error_theta = odometry_a_real - odometry_a;
    
    pos_absolute_past = pos_absolute_current;
    pos_odometry_past = pos_odometry_current;

    errores_d(i+1) = error_d;
    errores_a(i+1) = error_theta;


    addpoints(h1, i, error_d);
    addpoints(h3, i, error_theta);
    
    h2.Data = errores_d; 
    h4.Data = errores_a; 
    
    drawnow limitrate;
end
var_errores_d = var(errores_d);
var_errores_a = var(errores_a);

mean_errores_d = mean(errores_d);
mean_errores_a = mean(errores_a);
std_errores_d = std(errores_d);
std_errores_a = std(errores_a);

save('stats_error_motores_odometria.mat', 'var_errores_d', 'var_errores_a', 'mean_errores_d', 'mean_errores_a', 'std_errores_d', 'std_errores_a');