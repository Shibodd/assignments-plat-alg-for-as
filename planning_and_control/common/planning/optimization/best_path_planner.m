function [us, idx, cost] = best_path_planner(optimizer, x0, tgt_l, phidotdes, lminmax_alts)
    arguments
        optimizer (1,1) function_handle
        x0 (4,1) double
        tgt_l (:,1) double
        phidotdes (1,:)
        lminmax_alts (1,:) cell
    end
    
    idx = -1;
    cost = Inf;
    us = missing;

    for path_i=1:numel(lminmax_alts)
        [u, c] = optimizer(x0, tgt_l, phidotdes, lminmax_alts{path_i});

        if c < cost
            cost = c;
            us = u;
            idx = path_i;
        end
    end
end

