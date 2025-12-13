function path = a_star(map, start_pose, goal_pose, params)
    res = params.resolution;
    [rows, cols] = size(map);
    
    safety_margin = 0; 

    offset_x = (cols * res) / 2;
    offset_y = (rows * res) / 2;
    
    s_col = ceil((start_pose(1) + offset_x) / res);
    g_col = ceil((goal_pose(1) + offset_x) / res);
    s_row = rows - ceil((start_pose(2) + offset_y) / res) + 1;
    g_row = rows - ceil((goal_pose(2) + offset_y) / res) + 1;
    
    s_col = max(1, min(s_col, cols)); s_row = max(1, min(s_row, rows));
    g_col = max(1, min(g_col, cols)); g_row = max(1, min(g_row, rows));
    
    start_idx = sub2ind([rows, cols], s_row, s_col);
    goal_idx  = sub2ind([rows, cols], g_row, g_col);

    safe_map = map;
    if safety_margin > 0
        [obs_r, obs_c] = find(map == 0);
        for i = 1:length(obs_r)
            r_min = max(1, obs_r(i) - safety_margin);
            r_max = min(rows, obs_r(i) + safety_margin);
            c_min = max(1, obs_c(i) - safety_margin);
            c_max = min(cols, obs_c(i) + safety_margin);
            safe_map(r_min:r_max, c_min:c_max) = 0;
        end
    end
    
    if safe_map(start_idx) == 0 && map(start_idx) == 1
        safe_map(start_idx) = 1; 
    end
    if safe_map(goal_idx) == 0 && map(goal_idx) == 1
        safe_map(goal_idx) = 1;
    end

    if safe_map(start_idx) == 0 
        warning('Start maps to obstacle (or safety zone)'); path = []; return;
    end
    if safe_map(goal_idx) == 0
        warning('Goal maps to obstacle (or safety zone)'); path = []; return;
    end

    g_score = inf(rows, cols); f_score = inf(rows, cols);
    parent = zeros(rows, cols);
    g_score(start_idx) = 0;
    f_score(start_idx) = sqrt((s_col - g_col)^2 + (s_row - g_row)^2);
    
    open_set = [start_idx];
    offsets = [0 1 1; 0 -1 1; 1 0 1; -1 0 1; 1 1 1.41; 1 -1 1.41; -1 1 1.41; -1 -1 1.41];

    path_found = false;
    while ~isempty(open_set)
        [~, min_k] = min(f_score(open_set));
        current_idx = open_set(min_k);
        
        if current_idx == goal_idx
            path_found = true; break;
        end
        
        open_set(min_k) = [];
        [cy, cx] = ind2sub([rows, cols], current_idx);
        
        for k = 1:8
            ny = cy + offsets(k,1);
            nx = cx + offsets(k,2);
            
            if ny > 0 && ny <= rows && nx > 0 && nx <= cols && safe_map(ny, nx) == 1
                
                is_diagonal = (offsets(k,1) ~= 0) && (offsets(k,2) ~= 0);
                if is_diagonal
                    if map(cy, nx) == 0 || map(ny, cx) == 0
                        continue; 
                    end
                end
                
                neighbor_idx = sub2ind([rows, cols], ny, nx);
                tentative_g = g_score(current_idx) + offsets(k,3);
                
                if tentative_g < g_score(neighbor_idx)
                    parent(neighbor_idx) = current_idx;
                    g_score(neighbor_idx) = tentative_g;
                    f_score(neighbor_idx) = tentative_g + sqrt((nx - g_col)^2 + (ny - g_row)^2);
                    
                    if ~any(open_set == neighbor_idx)
                        open_set(end+1) = neighbor_idx;
                    end
                end
            end
        end
    end
    
    if path_found
        path = reconstruct_path(parent, goal_idx, rows, cols, res, offset_x, offset_y);
    else
        path = [];
    end
end

function path = reconstruct_path(parent, current_idx, rows, cols, res, off_x, off_y)
    path_indices = [current_idx];
    while parent(current_idx) ~= 0
        current_idx = parent(current_idx);
        path_indices = [current_idx, path_indices];
    end
    
    [py, px] = ind2sub([rows, cols], path_indices');
    
    wx = (px * res) - off_x;
    
    flipped_py = rows - py + 1; 
    wy = (flipped_py * res) - off_y;
    
    path = [wx, wy];
end