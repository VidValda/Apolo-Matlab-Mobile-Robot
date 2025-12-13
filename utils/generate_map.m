function map = generate_map()
    map_size = [35, 40];
    map = ones(map_size);
    
    num_blobs = 10;
    min_radius = 2;
    max_radius = 3;
    
    [X, Y] = meshgrid(1:map_size(2), 1:map_size(1));
    
    for i = 1:num_blobs
        cx = randi(map_size(2));
        cy = randi(map_size(1));
        r = randi([min_radius, max_radius]);
        dist_sq = (X - cx).^2 + (Y - cy).^2;
        map(dist_sq <= r^2) = 0;
    end

    center_x = map_size(2) / 2;
    center_y = map_size(1) / 2;
    map((X - center_x).^2 + (Y - center_y).^2 <= 6^2) = 1;

    map(1, :) = 0;
    map(end, :) = 0;
    map(:, 1) = 0;
    map(:, end) = 0;
end