function [bnd] = boundgen(frenet_obs, lmin, lmax)
    arguments
        frenet_obs (1,:) cell
        lmin (1,:) double
        lmax (1,:) double
    end

    N = numel(frenet_obs);
    assert(numel(lmax) == N || numel(lmin) == N, ...
        "lmax, lmin and frenet_obs should all have the same length." ...
    );

    bnd = cell(1, N);
    for i=1:N
        % Use a sweep line algorithm to compute the boundaries.
        infeasible_ranges = sweep_ranges([
            -inf, lmin(i); % lane min (as an obstacle)
            lmax(i), +inf; % lane max (as an obstacle)
            frenet_obs{i} % the actual obstacles
        ]);
        
        feasible_ranges = neg_range_bounds(infeasible_ranges);
        bnd{i} = reshape(feasible_ranges, 2, [])';
    end
end