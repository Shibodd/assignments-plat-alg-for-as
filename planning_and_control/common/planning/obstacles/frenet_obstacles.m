function frenet_obs = frenet_obstacles(obstacles, path_positions, path_normals, safety_dist)
    arguments
        obstacles (1,:) cell
        path_positions (2,:) double
        path_normals (2,:) double
        safety_dist (1,1) double = 0
    end
    
    obs_count = size(obstacles, 2);
    path_count = size(path_positions, 2);

    assert(isequal(size(path_normals), size(path_positions)), "Path positions and normals should have the same size.");

    frenet_obs = cell(path_count, 1);
    if numel(obstacles) > 0
        for i=1:path_count
            obs_frenet = vstackfi(obs_count, ...
                @(j) obstacles{j}.line_intersection(path_positions(:,i), path_normals(:,i)) ...
            );
            obs_frenet = obs_frenet(~any(isnan(obs_frenet), 2), :);
            obs_frenet = obs_frenet + [-safety_dist, safety_dist];
    
            frenet_obs{i} = obs_frenet;
        end
    end
end