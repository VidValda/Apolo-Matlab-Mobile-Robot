dt = 0.1;
T_max = 40;

robotName = 'Marvin';
laserName = 'LMS100';

goal_poses = [
    2,  0,  0;
    3,  -2,  pi/2;
    3,  -3, pi/2;
    0,  -2, pi
];

map = generate_map();

ROBOT_TYPE = 'DIFFERENTIAL';
PLANNER_TYPE = 'A*';
ESTIMATOR_TYPE = 'EKF';
CONTROLLER_TYPE = 'PID';
SENSOR_TYPE = 'BEACONS';

switch ROBOT_TYPE
    case 'DIFFERENTIAL'
        robot_params.sensor_y_offset = 0.1;
        f_robot = @differential_robot;
end

switch PLANNER_TYPE
    case 'A*'
        planner_params.heuristic = 'euclidean';
        planner_params.max_iterations = 1000;
        planner_params.tolerance = 0.01;
        planner_params.resolution = 0.2;
        f_planner = @a_star;
end

switch CONTROLLER_TYPE
    case 'PID'
        controller_params.Kp_v = 1.0;
        controller_params.Kp_w = 2.0;
        f_controller = @ctrl_pid;
end

switch ESTIMATOR_TYPE
    case 'EKF'
        estimator_params.Q = diag([0.1, 0.1, 0.05]);
        estimator_params.R = diag([0.5, 0.5, 0.1]);
        estimator_params.P = eye(3);
        estimator_params.x_true = [0; 0; 0];
        f_estimator = @ekf;
end

switch SENSOR_TYPE
    case 'BEACONS'
        sensor_params.beacons_pos = [
           -3.9, 3.9;
            3.9, 3.9;
        ];
        sensor_params.correction_factor = 0.02;
        f_sensors = @beacons_sensor;

end