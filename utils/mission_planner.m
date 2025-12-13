function [ref_out, path_concat] = mission_planner(planner_func, map, start, goals, p_params, dt)
    full_path_x = [];
    full_path_y = [];
    current_start = start(1:2);
    
    avg_speed = 0.3; 

    for i = 1:size(goals, 1)
        current_goal = goals(i, 1:2)';
        
        segment_path = planner_func(map, current_start, current_goal, p_params);
        
        if isempty(segment_path)
            warning('Planner failed for goal %d. Skipping segment.', i);
            continue;
        end
        
        full_path_x = [full_path_x; segment_path(:,1)];
        full_path_y = [full_path_y; segment_path(:,2)];
        
        current_start = current_goal;
    end
    
    path_concat = [full_path_x, full_path_y];
    
    if isempty(path_concat)
        error('Mission Failed: No path could be generated to any goal.');
    end
    
    dist_sq = sum(diff(path_concat).^2, 2);
    is_duplicate = [false; dist_sq < 1e-6];
    path_concat(is_duplicate, :) = [];
    full_path_x = path_concat(:,1);
    full_path_y = path_concat(:,2);

    dists = sqrt(sum(diff(path_concat).^2, 2));
    time_points = [0; cumsum(dists) / avg_speed];
    
    [time_points, unique_idx] = unique(time_points, 'stable');
    full_path_x = full_path_x(unique_idx);
    full_path_y = full_path_y(unique_idx);
    
    t_vec = 0:dt:max(time_points);
    
    ref_out.time = t_vec;
    ref_out.x  = interp1(time_points, full_path_x, t_vec, 'pchip');
    ref_out.y  = interp1(time_points, full_path_y, t_vec, 'pchip');
    
    dx = gradient(ref_out.x);
    dy = gradient(ref_out.y);
    ref_out.th = atan2(dy, dx);
end