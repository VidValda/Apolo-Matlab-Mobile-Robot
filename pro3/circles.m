clear; clc; close all;

robotName = 'Marvin';
dt = 0.5;

start_x = 0;
start_y = 2;

apoloPlaceMRobot(robotName, [start_x, start_y, 0], 0);
apoloResetOdometry(robotName, [start_x, start_y, 0]);
apoloUpdate();

radio = 2;
vueltas = 5;
distanciatotal = 2*pi*radio;
velocidadL = 1; 

numerodepasos = ceil(distanciatotal/(velocidadL*dt)); 
velocidadA = -2*pi/(numerodepasos*dt);
total_steps = ceil(numerodepasos * vueltas);

figure('Name', 'Repeatability Test Results', 'NumberTitle', 'off');

subplot(2,1,1);
hDist = animatedline('Color', 'b', 'LineWidth', 1.5);
yline(2, 'r--', 'Target Radius');
title('Radial Consistency (Target = 2.0)');
ylabel('Distance from Center');
grid on;

subplot(2,1,2);
hPath = animatedline('Color', 'k', 'LineWidth', 1);
axis equal; 
title('Robot Trajectory Trace');
xlabel('X'); ylabel('Y');
grid on;

for i = 1:total_steps
    pos_absolute_current = apoloGetOdometry(robotName);
    
    current_radial_dist = sqrt(pos_absolute_current(1)^2 + pos_absolute_current(2)^2);

    apoloMoveMRobot(robotName, [velocidadL, velocidadA], dt);
    
    apoloUpdate(); 
    pause(dt);
    
    addpoints(hDist, i, current_radial_dist);
    addpoints(hPath, pos_absolute_current(1), pos_absolute_current(2));
    
    if mod(i, 10) == 0
        drawnow limitrate;
    end
end