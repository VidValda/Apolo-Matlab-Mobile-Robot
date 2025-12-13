close all; clear; clc;
addpath(genpath('modules'));
addpath('utils');
run('config.m');
apoloPlaceMRobot(robotName, [0, 0, 0.2], 0);
apoloResetOdometry(robotName, [0, 0, 0]);
apoloUpdate();

x_est = [0; 0; 0];
P = eye(3);
[ref_data, full_path_xy] = mission_planner(f_planner, map, x_est, goal_poses, planner_params, dt);
N_steps = length(ref_data.time);

pixel_path = full_path_xy * 5; 

pixel_path(:,1) = pixel_path(:,1) + size(map, 2)/2;
pixel_path(:,2) = size(map, 1)/2 - pixel_path(:,2); 

imshow(map)
hold on;
plot(pixel_path(:,1), pixel_path(:,2), 'r-', 'LineWidth', 2);
hold off;
u_k = zeros([2,N_steps]);
for k = 1:N_steps

    ref_state.x  = ref_data.x(k);
    ref_state.y  = ref_data.y(k);
    ref_state.th = ref_data.th(k);
        
    u = f_controller(x_est, ref_state, controller_params);
    u_k(:,k) = u;
    
    f_robot(robotName,u, dt);
    
    [z,beacons_xy] = f_sensors(laserName,sensor_params);
    
    [x_est, P] = f_estimator(x_est, u, z, beacons_xy, P, dt, estimator_params);
    %absolute_pos = apoloGetLocationMRobot(robotName);
    %x_est = [absolute_pos(1),absolute_pos(2),absolute_pos(4)];
end
