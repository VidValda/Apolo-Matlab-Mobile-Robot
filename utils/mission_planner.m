function [ref_out, path_concat] = mission_planner(planner_func, map, start, goals, p_params, dt)

    hold_time = 0;
    if isfield(p_params,'hold_time'); hold_time = p_params.hold_time; end

    avg_speed = 0.5;

    full_path_x = [];
    full_path_y = [];
    time_points = [];

    current_start = start(1:2);
    t = 0;

    for i = 1:size(goals, 1)
        current_goal = goals(i, 1:2)';

        segment_path = planner_func(map, current_start, current_goal, p_params);
        if isempty(segment_path)
            warning('Planner failed for goal %d. Skipping segment.', i);
            continue;
        end

        % Para no duplicar el primer punto del segmento (si ya coincide con el último anterior)
        if ~isempty(full_path_x)
            segment_path = segment_path(2:end,:);
        end

        % Ir agregando puntos y tiempo por distancia/velocidad
        for j = 1:size(segment_path,1)
            px = segment_path(j,1);
            py = segment_path(j,2);

            if isempty(full_path_x)
                % primer punto
                full_path_x(end+1,1) = px;
                full_path_y(end+1,1) = py;
                time_points(end+1,1) = t;
            else
                dx = px - full_path_x(end);
                dy = py - full_path_y(end);
                d  = hypot(dx,dy);

                t = t + d/avg_speed;

                full_path_x(end+1,1) = px;
                full_path_y(end+1,1) = py;
                time_points(end+1,1) = t;
            end
        end

        % "Quedarse" en el goal entre segmentos: duplico el último punto y sumo 5s
        if (i < size(goals,1)) && (hold_time > 0) && ~isempty(full_path_x)
            t = t + hold_time;
            full_path_x(end+1,1) = full_path_x(end);
            full_path_y(end+1,1) = full_path_y(end);
            time_points(end+1,1) = t;
        end

        current_start = current_goal;
    end

    path_concat = [full_path_x, full_path_y];

    if isempty(path_concat)
        error('Mission Failed: No path could be generated to any goal.');
    end

    % Vector de referencia temporal
    t_vec = 0:dt:time_points(end);

    ref_out.time = t_vec;
    ref_out.x = interp1(time_points, full_path_x, t_vec, 'linear');
    ref_out.y = interp1(time_points, full_path_y, t_vec, 'linear');

    dx = gradient(ref_out.x);
    dy = gradient(ref_out.y);
    ref_out.th = atan2(dy, dx);
end
