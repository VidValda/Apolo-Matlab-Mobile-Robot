close all; clear; clc;
addpath(genpath('modules'));
addpath('utils');
run('config.m');
apoloPlaceMRobot(robotName, [0, 0, 0.2], 0);
apoloResetOdometry(robotName, [0, 0, 0]);
apoloUpdate();
last_odo = apoloGetOdometry(robotName);

x_est = [0; 0; 0];
P = diag([0.001,0.001,0.001]);
[ref_data, full_path_xy] = mission_planner(f_planner, map, x_est, goal_poses, planner_params, dt);
N_steps = length(ref_data.time);

viz = init_visualizer(map, full_path_xy);
vis_step = 5;

history.x_est = zeros(3, N_steps);
history.x_true = zeros(3, N_steps);
history.sigma = zeros(3, N_steps);
history.time = ref_data.time;

for k = 1:N_steps

    ref_state.x  = ref_data.x(k);
    ref_state.y  = ref_data.y(k);
    ref_state.th = ref_data.th(k);
        
    u = f_controller(x_est, ref_state, controller_params);
    
    f_robot(robotName,u, dt);
    [u_odometry, curr_odo] = odometry(robotName, last_odo, dt);
    last_odo = curr_odo;
    
    abs_pos = apoloGetLocationMRobot(robotName); %GROUD-TRUTH; JUST FOR GRAPHS
    x_true = [abs_pos(1); abs_pos(2); abs_pos(4)]; %GROUD-TRUTH; JUST FOR GRAPHS
    
    [z,beacons_xy] = f_sensors(laserName,sensor_params);
    
    [x_est, P] = f_estimator(x_est, u_odometry, z, beacons_xy, P, dt, estimator_params);
    
    history.x_est(:, k) = x_est;
    history.x_true(:, k) = x_true;
    history.sigma(:, k) = 3 * sqrt(diag(P));

    if mod(k, vis_step) == 0
        update_visualizer(viz, x_est,P);
    end
end


figure('Name', 'Errores de Estimación e Incertidumbre', 'Color', 'w');

titulos = {'Posición X', 'Posición Y', 'Orientación \theta'};
unidades = {'metros', 'metros', 'radianes'};

errors = history.x_true - history.x_est;

errors(3,:) = atan2(sin(errors(3,:)), cos(errors(3,:)));

for i = 1:3
    subplot(3, 1, i); hold on; grid on;
    
    plot(history.time, history.sigma(i,:), 'r--', 'LineWidth', 1.5);
    
    plot(history.time, -history.sigma(i,:), 'r--', 'LineWidth', 1.5);
    
    plot(history.time, errors(i,:), 'b-', 'LineWidth', 1.5);
    
    ylabel(['Error (' unidades{i} ')']);
    title(['Error en ' titulos{i} ' vs. 3\sigma Bounds']);
    legend('Limite +3\sigma', 'Limite -3\sigma', 'Error Real');
end
xlabel('Tiempo (s)');