function path = a_star(map, start_pose, goal_pose, params)

    [rows, cols] = size(map);
    resX = params.Xmax / (rows-1);
    resY = params.Ymax / (cols-1);
    
    world2grid = @(x,y) deal( ...
        max(1, min(rows, 1 + floor(x/resX))), ...
        max(1, min(cols, 1 + floor((params.Ymax - y)/resY))) );
    
    grid2world = @(row,col) deal( ...
        (row-1)*resX, ...
        params.Ymax - (col-1)*resY );

    [s_row, s_col] = world2grid(double(start_pose(1)), double(start_pose(2)));
    [g_row, g_col] = world2grid(double(goal_pose(1)), double(goal_pose(2)));

    s_idx_orig = s_row + (s_col-1)*rows;
    g_idx_orig = g_row + (g_col-1)*rows;

    if map(s_idx_orig)==0 || map(g_idx_orig)==0
        fprintf("WARN: Start or Goal falls on static obstacle.\n");
        path = []; return;
    end

    occ = (map == 0);
    dist_px = bwdist(occ);
    
    inflate_px = max(1, ceil(params.lethal_radius / min(resX,resY)));
    occ_inf = imdilate(occ, strel('disk', inflate_px, 0));
    
    costMapRaw = 1 + params.cost_gain * exp(-(dist_px * min(resX,resY)) / 0.4);
    costMapRaw(occ_inf) = Inf;

    [~, nearest_free_idx] = bwdist(~occ_inf);
    
    if occ_inf(s_idx_orig)
        [s_row, s_col] = ind2sub([rows, cols], nearest_free_idx(s_idx_orig));
    end
    if occ_inf(g_idx_orig)
        [g_row, g_col] = ind2sub([rows, cols], nearest_free_idx(g_idx_orig));
    end

    rows_pad = rows + 2;
    cols_pad = cols + 2;
    costMap = inf(rows_pad, cols_pad);
    costMap(2:end-1, 2:end-1) = costMapRaw;

    s_row_pad = s_row + 1; s_col_pad = s_col + 1;
    g_row_pad = g_row + 1; g_col_pad = g_col + 1;

    start_idx = uint32(s_row_pad + (s_col_pad-1)*rows_pad);
    goal_idx  = uint32(g_row_pad + (g_col_pad-1)*rows_pad);

    [parent_map, found, iter] = run_astar_heap(costMap, start_idx, goal_idx, g_row_pad, g_col_pad, rows_pad, params.max_iterations);

    if ~found
        path = []; return;
    end

    idx_path = double(goal_idx);
    cur = double(goal_idx);
    start_idx_dbl = double(start_idx);

    while cur ~= start_idx_dbl
        cur = double(parent_map(cur));
        if cur == 0, path = []; return; end
        idx_path(end+1) = cur;
    end
    
    [pr_pad, pc_pad] = ind2sub([rows_pad, cols_pad], flip(idx_path));
    
    path = zeros(numel(pr_pad), 2);
    for k = 1:numel(pr_pad)
        [path(k,1), path(k,2)] = grid2world(pr_pad(k)-1, pc_pad(k)-1);
    end
    
    fprintf("Done. Path nodes: %d. Iterations: %d / %d\n", size(path,1), iter, params.max_iterations);
end

function [parent, found, iter] = run_astar_heap(costMap, start_idx, goal_idx, gr, gc, rows, max_iter)
    num_nodes = numel(costMap);
    g = inf(num_nodes, 1);
    parent = zeros(num_nodes, 1, 'uint32');
    closed = false(num_nodes, 1);
    
    heap_f   = zeros(num_nodes, 1);
    heap_id  = zeros(num_nodes, 1, 'uint32');
    heap_pos = zeros(num_nodes, 1, 'uint32');
    
    offsets = double([-1, 1, rows, -rows, rows-1, -rows-1, rows+1, -rows+1]);
    d_row = double([-1, 1, 0, 0, -1, -1, 1, 1]);
    d_col = double([0, 0, 1, -1, 1, -1, 1, -1]);
    move_costs = [1, 1, 1, 1, 1.4142, 1.4142, 1.4142, 1.4142];
    
    g(start_idx) = 0;
    n_heap = 1;
    
    [sr, sc] = ind2sub(size(costMap), double(start_idx));
    heap_f(1) = 1.2 * hypot(sr - double(gr), sc - double(gc));
    heap_id(1) = start_idx;
    heap_pos(start_idx) = 1;

    found = false;
    iter = 0;
    
    while n_heap > 0
        iter = iter + 1;
        
        if mod(iter, 5000) == 0
             fprintf('%d / %d\n', iter, max_iter);
             drawnow;
        end
        
        if iter > max_iter, break; end
        
        current = heap_id(1);
        heap_pos(current) = 0; 
        
        if current == goal_idx
            found = true; break;
        end
        
        last_node = heap_id(n_heap);
        last_f    = heap_f(n_heap);
        n_heap    = n_heap - 1;
        
        if n_heap > 0
            idx = 1;
            heap_id(1) = last_node;
            heap_f(1)  = last_f;
            heap_pos(last_node) = 1;
            
            while true
                left = 2 * idx;
                if left > n_heap, break; end
                right = left + 1;
                smallest = idx;
                
                if heap_f(left) < heap_f(smallest), smallest = left; end
                if right <= n_heap && heap_f(right) < heap_f(smallest), smallest = right; end
                
                if smallest ~= idx
                    sid = heap_id(smallest); sf = heap_f(smallest);
                    heap_id(smallest) = heap_id(idx); heap_f(smallest) = heap_f(idx);
                    heap_id(idx) = sid; heap_f(idx) = sf;
                    
                    heap_pos(heap_id(idx)) = idx;
                    heap_pos(heap_id(smallest)) = smallest;
                    idx = smallest;
                else
                    break;
                end
            end
        end

        closed(current) = true;
        current_g = g(current);
        
        [cr, cc] = ind2sub(size(costMap), double(current));

        for k = 1:8
            neighbor = current + offsets(k); 
            
            if closed(neighbor), continue; end
            
            c_val = costMap(neighbor);
            if isinf(c_val), continue; end
            
            tentative_g = current_g + move_costs(k) * c_val;
            
            if tentative_g < g(neighbor)
                g(neighbor) = tentative_g;
                parent(neighbor) = current;
                
                nr = cr + d_row(k);
                nc = cc + d_col(k);
                f_val = tentative_g + 1.2 * hypot(nr - double(gr), nc - double(gc));
                
                pos = heap_pos(neighbor);
                
                if pos == 0
                    n_heap = n_heap + 1;
                    curr_h = n_heap;
                    heap_id(curr_h) = neighbor;
                    heap_f(curr_h)  = f_val;
                    heap_pos(neighbor) = curr_h;
                else
                    curr_h = pos;
                    heap_f(curr_h) = f_val;
                end
                
                while curr_h > 1
                    parent_h = floor(curr_h / 2);
                    if heap_f(curr_h) < heap_f(parent_h)
                        tid = heap_id(curr_h); tf = heap_f(curr_h);
                        heap_id(curr_h) = heap_id(parent_h); heap_f(curr_h) = heap_f(parent_h);
                        heap_id(parent_h) = tid; heap_f(parent_h) = tf;
                        
                        heap_pos(heap_id(curr_h)) = curr_h;
                        heap_pos(heap_id(parent_h)) = parent_h;
                        curr_h = parent_h;
                    else
                        break;
                    end
                end
            end
        end
    end
end