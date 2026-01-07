dt = 0.2;
T_max = 40;

robotName = 'Marvin';
laserName = 'LMS100';

goal_poses = [
    %10,  10, 0;
    %18,  18, pi/2;
    %17,  3, pi/2;
    %25,  18.5, pi;
    %22,  3, 0;
    %32,  16.5, pi/2;
    32,  5, pi/2;
    %5.5,  5.5, pi;
];

sensor_y_offset = 0.1;

mission_params.hold_time = 5.0;      % segundos
mission_params.goal_tol  = 0.25;     % m (tolerancia para “llegué al goal”)
mission_params.th_tol    = deg2rad(20); % opcional, tolerancia angular
planner_params.hold_time = 10;   % segundos de espera en cada goal (entre segmentos)

Xmax = 40; 
Ymax = 20;

S = load('models/Factory_map_no_cone.mat'); % Cargar mapa de /model
fn = fieldnames(S);
bin = S.(fn{1});

% bin==0 es obstáculo  -> map: 1 libre, 0 obstáculo
map = double(bin ~= 0);

% Queremos filas = X (alto=40m) y cols = Y (ancho=20m)
if size(map,1) < size(map,2)
    map = map';     % "para" el mapa (lo pone como tu foto correcta)
end

% X=0 abajo (porque MATLAB pone fila 1 arriba)
%map = flipud(map);

planner_params.Xmax = Xmax;
planner_params.Ymax = Ymax;


figure; imagesc(map); axis equal tight; colormap gray;
title('map (blanco=libre, negro=obstáculo)');
set(gca,'YDir','normal');


ROBOT_TYPE = 'DIFFERENTIAL';
PLANNER_TYPE = 'A*';
ESTIMATOR_TYPE = 'EKF';
CONTROLLER_TYPE = 'PID';
SENSOR_TYPE = 'BEACONS';
CONTROLLER_TYPE2 = 'REACTIVE';

switch ROBOT_TYPE
    case 'DIFFERENTIAL'
        robot_params.sensor_y_offset = sensor_y_offset;
        f_robot = @differential_robot;
end

switch PLANNER_TYPE
    case 'A*'
        planner_params.resolution = 0.01;

        planner_params.origin_x = 0;     % coordenada mundo del píxel (col=cols)
        planner_params.origin_y = 0;     % coordenada mundo del píxel (row=rows)
        planner_params.debug = true;

        planner_params.lethal_radius = 0.08;   % colisión real (lo que antes era inflate_radius)
        planner_params.cost_gain     = 80; % penalización cerca de obstáculo
        planner_params.max_iterations = 3000000; % más realista que 1000
        f_planner = @a_star;                   % queda igual
end

switch CONTROLLER_TYPE
    case 'PID'
        controller_params.Kp_v = 1.5;
        controller_params.Kp_w = 1.3;
        f_controller = @ctrl_pid;
end


switch CONTROLLER_TYPE2
    case 'REACTIVE'
        % -------- Reactive (LaserData) params --------
        reactive_params.range_min = 0.05;
        reactive_params.range_max = 10.0;
        reactive_params.w_escape = 0.8;   % giro fuerte en emergencia

        % LMS100 típico: ~270° y 541 medidas (depende de xml)
        % Si no estás segura, empieza con [-pi/2, pi/2] y ajusta
        reactive_params.ang_min = -3*pi/4;
        reactive_params.ang_max =  3*pi/4;
        
        reactive_params.front_half_angle = deg2rad(25);  % +/- 20° frente
        reactive_params.side_angle       = deg2rad(120);  % costados
        
        reactive_params.d_slow = 0.8;   % empieza a reaccionar
        reactive_params.d_stop = 0.25;   % se para si está muy cerca
        
        reactive_params.v_slow = 0.15;  % velocidad "de seguridad"
        
        reactive_params.k_w = 0.9;      % giro evasivo
        
        reactive_params.v_min = -0.15;
        reactive_params.v_max = 0.6;    % ajusta a tu robot
        reactive_params.w_max = 1.8;
        
        reactive_params.d_min = 0.85;      % 0.8 distancia mínima deseada (seguridad)
        
        
        reactive_params.escape_steps = 6; % 12 pasos * dt=0.1 => 1.2s escapando
        reactive_params.v_back = -0.08;    % retroceso suave (m/s) para despegarse
        


end 
        

switch ESTIMATOR_TYPE
    case 'EKF'
        % estimator_params.Q = diag([2.2188e-05,9.6568e-07]);
        % estimator_params.R = diag([2.5018e-04,3.7769e-04]);
        estimator_params.Q = diag([0.2786,0.0818]*1.0e-03*3);
        estimator_params.R = diag([2.2969,0.0127]*3);
        estimator_params.sensor_y_offset = sensor_y_offset;
        estimator_params.correction_factor = 0.02;
        f_estimator = @ekf;
end

switch SENSOR_TYPE
    case 'BEACONS'
        sensor_params.beacons_pos = [
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
        sensor_params.correction_factor = 0.02;
        f_sensors = @beacons_sensor;

end