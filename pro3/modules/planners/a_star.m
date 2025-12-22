function path = a_star(map, start_pose, goal_pose, params)

    [rows, cols] = size(map);

    resX = params.Xmax/(rows-1);
    resY = params.Ymax/(cols-1);
    res = min(resX, resY);
    
    world2grid = @(x,y) deal( ...
        max(1, min(rows, 1 + floor(x/resX))), ...
        max(1, min(cols, 1 + floor((params.Ymax - y)/resY))) );
    
    grid2world = @(row,col) deal( ...
        (row-1)*resX, ...
        params.Ymax - (col-1)*resY );


    sx = double(start_pose(1)); sy = double(start_pose(2));
    gx = double(goal_pose(1));  gy = double(goal_pose(2));

    [s_row, s_col] = world2grid(sx,sy);
    [g_row, g_col] = world2grid(gx,gy);

    start_idx = sub2ind([rows, cols], s_row, s_col);
    goal_idx  = sub2ind([rows, cols], g_row, g_col);

    if map(start_idx)==0 || map(goal_idx)==0
        path = []; return;
    end

    % ===== Inflado duro + costmap suave (RUTH5) =====
    occ     = (map == 0);

    dist_px = bwdist(occ);
    dist_m  = dist_px * min(resX,resY);   % metros conservador
    
    inf_r = params.lethal_radius;
    gain  = params.cost_gain;
    maxit = params.max_iterations;
    
    decay = 0.4;
    
    inflate_px = max(1, ceil(inf_r / min(resX,resY)));
    occ_inf = imdilate(occ, strel('disk', inflate_px, 0));
    
    costMap = 1 + gain * exp(-dist_m / decay);
    costMap(occ_inf) = Inf;

   





    fprintf("START grid=(r=%d,c=%d), GOAL grid=(r=%d,c=%d)\n", s_row, s_col, g_row, g_col);
    fprintf("Lethal radius: %.2f, Cost gain: %.1f\n", inf_r, gain);

    % --- Reubicar solo si caen en inflado ---
    if occ_inf(s_row,s_col)
        [s_row, s_col] = nearestFreeCell(s_row, s_col, occ_inf);
    end
    if occ_inf(g_row,g_col)
        [g_row, g_col] = nearestFreeCell(g_row, g_col, occ_inf);
    end

    start_idx = sub2ind([rows, cols], s_row, s_col);
    goal_idx  = sub2ind([rows, cols], g_row, g_col);

    fprintf("Inflated cells: %d / %d\n", nnz(occ_inf), numel(occ_inf));

    % --- Chequeo conectividad ---
    free = ~occ_inf;
    CC = bwlabel(free, 8);
    if CC(s_row,s_col)==0 || CC(g_row,g_col)==0 || CC(s_row,s_col) ~= CC(g_row,g_col)
        fprintf("No hay conectividad con este inflado (start y goal en componentes distintas)\n");
        path = [];
        return;
    end

    % ===== A* =====
    g = inf(rows, cols);
    f = inf(rows, cols);
    parent = zeros(rows, cols, 'uint32');
    closed = false(rows, cols);
    open_mask = false(rows, cols);
    
    g(start_idx) = 0;
    w = 1.2; % weighted A*
    f(start_idx) = w*hypot(double(s_row-g_row), double(s_col-g_col));
    
    neigh = [0 1 1; 0 -1 1; 1 0 1; -1 0 1; 1 1 1.4142; 1 -1 1.4142; -1 1 1.4142; -1 -1 1.4142];
    
    open_cap = 200000;                 % capacidad inicial (200k)
    open_list = zeros(open_cap,1,'uint32');

    open_count = 1;
    open_list(1) = uint32(start_idx);
    open_mask(start_idx) = true;
    
    iter = 0;
    found = false;
    
    while open_count > 0
        iter = iter + 1;
        if iter > maxit
            fprintf("A*: alcanzó max_iterations=%d\n", maxit);
            break;
        end
    
        % mínimo f SOLO dentro de open_list (vector)
        % más rápido: solo mira esa porción (sin crear cur_candidates)
        vals = f(open_list(1:open_count));
        [~,kmin] = min(vals);

        current = open_list(kmin);
        
        % "remove" sin costo: swap con el último
        open_list(kmin) = open_list(open_count);
        open_count = open_count - 1;
        open_mask(current) = false;
    
        if closed(current)
            continue;
        end
    
        if current == goal_idx
            found = true;
            break;
        end
    
        closed(current) = true;
    
        [cy,cx] = ind2sub([rows, cols], double(current));
    
        for i = 1:8
            ny = cy + neigh(i,1); nx = cx + neigh(i,2);
            if ny<1||ny>rows||nx<1||nx>cols, continue; end
            if occ_inf(ny,nx), continue; end
    
            nidx = sub2ind([rows, cols], ny, nx);
            if closed(nidx), continue; end
    
            tentative_g = g(current) + neigh(i,3) * costMap(nidx);
    
            if tentative_g < g(nidx)
                g(nidx) = tentative_g;
                f(nidx) = tentative_g + w*hypot(double(ny-g_row), double(nx-g_col));
                parent(nidx) = uint32(current);
    
                if ~open_mask(nidx)
                    open_count = open_count + 1;
                    if open_count > numel(open_list)
                        % si se llenó, expandimos (raro si maxit razonable)
                        open_list = [open_list; zeros(numel(open_list),1,'uint32')]; %#ok<AGROW>
                    end
                    open_list(open_count) = uint32(nidx);
                    open_mask(nidx) = true;
                end
            end
        end

        if mod(iter,50000)==0
            fprintf("iter=%d open=%d g=%.1f\n", iter, open_count, g(current));
        end

    end


    
    



    if ~found, path = []; return; end

    % --- Reconstrucción ---
    idx_path = double(goal_idx); 
    cur = double(goal_idx);
    start_idx = double(start_idx);  % para comparar limpio

    while cur ~= start_idx
        cur = double(parent(cur));
        if cur == 0, path = []; return; end
        idx_path = [cur; idx_path];
    end

    [pr, pc] = ind2sub([rows, cols], idx_path);
    path = zeros(numel(pr), 2);
    for k = 1:numel(pr)
        [path(k,1), path(k,2)] = grid2world(pr(k), pc(k));
    end
end

function [rr, cc] = nearestFreeCell(r, c, occ_inf)
    free = ~occ_inf;
    if free(r,c), rr=r; cc=c; return; end
    [R,C] = find(free);
    [~,k] = min((R-r).^2 + (C-c).^2);
    rr = R(k); cc = C(k);
end
