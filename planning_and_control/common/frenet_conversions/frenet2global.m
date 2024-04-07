function traj = frenet2global(xs, path_positions, path_normals)
    traj = path_positions + xs(1,:) .* path_normals;
end